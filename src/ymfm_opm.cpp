// BSD 3-Clause License
//
// Copyright (c) 2021, Aaron Giles
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "ymfm_opm.h"
#include "ymfm_fm.ipp"

namespace ymfm
{

//*********************************************************
//  OPM REGISTERS
//*********************************************************

//-------------------------------------------------
//  s_lfo_preload - preload values for the 15-bit
//  LFO coarse counter, indexed by the upper 4 bits
//  of the LFO rate
//-------------------------------------------------

// the counter counts up until it overflows, so a preload of
// 0x8000 - (1 << (15 - hi)) yields 2^(15-hi) ticks per LFO clock
//
// entry 0 is 0x1000 rather than the 0x0000 that formula gives, matching the
// LUT in IKAOPM; this makes the slowest 16 rates run 8/7 faster than figure
// 2.16 of the application manual specifies
static uint16_t const s_lfo_preload[16] =
{
	0x1000, 0x4000, 0x6000, 0x7000, 0x7800, 0x7c00, 0x7e00, 0x7f00,
	0x7f80, 0x7fc0, 0x7fe0, 0x7ff0, 0x7ff8, 0x7ffc, 0x7ffe, 0x7fff
};


//-------------------------------------------------
//  s_lfo_noise_shift - which 8 bits of the 16-bit
//  noise word the LFO value path picks up
//-------------------------------------------------

// the noise word is shifted into the phase accumulator serially, so which
// bits end up where the sawtooth's phase would be is a matter of how the
// load window lines up with the multiplier's; this rotation is the one that
// reproduces IKAOPM
static uint32_t const s_lfo_noise_shift = 15;

// and the load window opens a few phi1 after the noise timer's does, so when
// both land on the same frame this many of the bits the noise shifts in
// arrive too late for the LFO to see them
static uint32_t const s_lfo_noise_lag = 2;


//-------------------------------------------------
//  opm_registers - constructor
//-------------------------------------------------

opm_registers::opm_registers() :
	m_noise_lfsr(0),
	m_noise_flag(0),
	m_lfo_coarse(0),
	m_lfo_prescaler(0),
	m_lfo_fine(0),
	m_lfo_phase(0),
	m_noise_counter(0),
	m_noise_state(0),
	m_lfo_noise(0),
	m_lfo_am(0)
{
	// create the waveforms
	for (uint32_t index = 0; index < WAVEFORM_LENGTH; index++)
		m_waveform[0][index] = abs_sin_attenuation(index) | (bitfield(index, 9) << 15);

	// create the LFO waveforms; AM in bits 0-7, PM magnitude in bits 8-15 and
	// PM sign in bit 16. the chip emits PM as a sign plus a magnitude rather
	// than as a two's complement value, so the sign has to be kept out of the
	// depth multiply -- otherwise the negative half comes out a step off
	// waveforms are adjusted to match the pictures in the application manual
	for (uint32_t index = 0; index < LFO_WAVEFORM_LENGTH; index++)
	{
		// waveform 0 is a sawtooth
		uint32_t am = index ^ 0xff;
		uint32_t sign = bitfield(index, 7);
		uint32_t mag = sign ? (index ^ 0xff) : index;
		m_lfo_waveform[0][index] = am | (mag << 8) | (sign << 16);

		// waveform 1 is a square wave; its PM is not taken from the phase at
		// all, so the magnitude is the same on both halves
		am = bitfield(index, 7) ? 0 : 0xff;
		m_lfo_waveform[1][index] = am | (0x80 << 8) | (bitfield(index, 7) << 16);

		// waveform 2 is a triangle wave
		am = (bitfield(index, 7) ? (index << 1) : ((index ^ 0xff) << 1)) & 0xff;
		uint32_t pm = (bitfield(index, 6) ? am : ~am) & 0xff;
		sign = bitfield(pm, 7);
		mag = sign ? (0x100 - pm) : pm;
		m_lfo_waveform[2][index] = am | (mag << 8) | (sign << 16);

		// waveform 3 is noise; it is filled in dynamically
		m_lfo_waveform[3][index] = 0;
	}
}


//-------------------------------------------------
//  reset - reset to initial state
//-------------------------------------------------

void opm_registers::reset()
{
	std::fill_n(&m_regdata[0], REGISTERS, 0);

	// reset clears the noise LFSR and its timer on the chip; the all-zero
	// state is escaped by the feedback's zero detect, so starting there is
	// safe and makes the noise sequence reproducible from reset
	m_noise_lfsr = 0;
	m_noise_flag = 0;
	m_noise_counter = 0;

	// enable output on both channels by default
	m_regdata[0x20] = m_regdata[0x21] = m_regdata[0x22] = m_regdata[0x23] = 0xc0;
	m_regdata[0x24] = m_regdata[0x25] = m_regdata[0x26] = m_regdata[0x27] = 0xc0;
}


//-------------------------------------------------
//  save_restore - save or restore the data
//-------------------------------------------------

void opm_registers::save_restore(ymfm_saved_state &state)
{
	state.save_restore(m_lfo_coarse);
	state.save_restore(m_lfo_prescaler);
	state.save_restore(m_lfo_fine);
	state.save_restore(m_lfo_phase);
	state.save_restore(m_lfo_am);
	state.save_restore(m_noise_lfsr);
	state.save_restore(m_noise_flag);
	state.save_restore(m_noise_counter);
	state.save_restore(m_noise_state);
	state.save_restore(m_lfo_noise);
	state.save_restore(m_regdata);
}


//-------------------------------------------------
//  operator_map - return an array of operator
//  indices for each channel; for OPM this is fixed
//-------------------------------------------------

void opm_registers::operator_map(operator_mapping &dest) const
{
	// Note that the channel index order is 0,2,1,3, so we bitswap the index.
	//
	// This is because the order in the map is:
	//    carrier 1, carrier 2, modulator 1, modulator 2
	//
	// But when wiring up the connections, the more natural order is:
	//    carrier 1, modulator 1, carrier 2, modulator 2
	static const operator_mapping s_fixed_map =
	{ {
		operator_list(  0, 16,  8, 24 ),  // Channel 0 operators
		operator_list(  1, 17,  9, 25 ),  // Channel 1 operators
		operator_list(  2, 18, 10, 26 ),  // Channel 2 operators
		operator_list(  3, 19, 11, 27 ),  // Channel 3 operators
		operator_list(  4, 20, 12, 28 ),  // Channel 4 operators
		operator_list(  5, 21, 13, 29 ),  // Channel 5 operators
		operator_list(  6, 22, 14, 30 ),  // Channel 6 operators
		operator_list(  7, 23, 15, 31 ),  // Channel 7 operators
	} };
	dest = s_fixed_map;
}


//-------------------------------------------------
//  write - handle writes to the register array
//-------------------------------------------------

bool opm_registers::write(uint16_t index, uint8_t data, uint32_t &channel, uint32_t &opmask)
{
	assert(index < REGISTERS);

	// LFO AM/PM depth are written to the same register (0x19);
	// redirect the PM depth to an unused neighbor (0x1a)
	if (index == 0x19)
		m_regdata[index + bitfield(data, 7)] = data;
	else if (index != 0x1a)
		m_regdata[index] = data;

	// writing the LFO rate reloads the coarse counter, so the LFO stops
	// advancing entirely if writes keep arriving faster than it can overflow
	if (index == 0x18)
		m_lfo_coarse = s_lfo_preload[bitfield(data, 4, 4)];

	// handle writes to the key on index
	if (index == 0x08)
	{
		channel = bitfield(data, 0, 3);
		opmask = bitfield(data, 3, 4);
		return true;
	}
	return false;
}


//-------------------------------------------------
//  step_noise - advance the noise LFSR by one bit
//-------------------------------------------------

void opm_registers::step_noise()
{
	// feedback is bit 2 XORed against the bit that fell off the bottom one
	// step earlier; expanded over time that is the same x^17 + x^14 sequence
	// an explicit 17-bit LFSR gives, with the extra bit of state living in
	// m_noise_flag. the chip breaks the all-zero lockup by forcing a 1
	// rather than by using XNOR feedback
	uint32_t feedback = (m_noise_flag ^ bitfield(m_noise_lfsr, 2)) |
		(m_noise_lfsr == 0 && m_noise_flag == 0);
	m_noise_flag = bitfield(m_noise_lfsr, 0);
	m_noise_lfsr = (m_noise_lfsr >> 1) | (feedback << 15);
}


//-------------------------------------------------
//  clock_noise_and_lfo - clock the noise and LFO,
//  handling clock division, depth, and waveform
//  computations
//-------------------------------------------------

int32_t opm_registers::clock_noise_and_lfo()
{
	// the noise LFSR is a 16-bit register that rotates once per phi1; since
	// an internal frame is 16 phi1 it comes all the way back around every
	// frame, so its contents only actually change during the frames where
	// the noise timer opens the feedback path -- and then every one of the
	// 16 bits is replaced in one go
	//
	// the LFO is divided down in three stages that run off the same frame
	// clock: a 4-bit prescaler, a 15-bit coarse counter preloaded from the
	// upper 4 bits of the rate, and a 4-bit fine counter decoded against the
	// lower 4 bits. both the noise and the LFO are stepped a frame at a time
	// here because in waveform 3 the LFO latches the noise register, and
	// which of the sample's two frames that happens in decides whether it
	// sees the value from before or after the noise advances
	uint32_t freq = noise_frequency();
	uint32_t rate = lfo_rate();
	for (int frame = 0; frame < 2; frame++)
	{
		// run the divider first so we know whether this frame clocks the LFO
		bool latch = false;
		if (++m_lfo_prescaler == 16)
		{
			m_lfo_prescaler = 0;

			// the coarse counter reloads on overflow, giving 2^(15-hi) ticks
			// -- 8*2^(15-hi) samples -- per LFO clock
			if (++m_lfo_coarse >= 0x8000)
			{
				m_lfo_coarse = s_lfo_preload[bitfield(rate, 4, 4)];

				// each coarse carry also steps the fine counter; decoding it
				// against the low 4 bits of the rate inserts an extra LFO
				// clock one frame later, adding lo extra clocks per 16
				// carries, which is what makes the average rate (16+lo)/16
				// times the coarse rate
				uint32_t step = 1;
				if (bitfield(m_lfo_fine, 0) == 0)
					step += bitfield(rate, 3);
				else if (bitfield(m_lfo_fine, 1) == 0)
					step += bitfield(rate, 2);
				else if (bitfield(m_lfo_fine, 2) == 0)
					step += bitfield(rate, 1);
				else if (bitfield(m_lfo_fine, 3) == 0)
					step += bitfield(rate, 0);
				m_lfo_fine = (m_lfo_fine + 1) & 15;

				// the extra clock lands within the same sample as the coarse
				// one, so the phase can advance by 2 at once
				m_lfo_phase = (m_lfo_phase + step) & 0xff;
				latch = true;
			}
		}

		// the 5-bit noise timer counts once per frame and matches for the
		// whole of the frame it lands on
		bool update = (m_noise_counter == freq);
		m_noise_counter = update ? 0 : (m_noise_counter + 1);

		// a frame with no feedback running is 16 rotations of a 16-bit
		// register, which leaves it exactly where it started
		if (!update)
		{
			if (latch)
				m_lfo_noise = m_noise_lfsr;
			continue;
		}

		// the LFO's load window opens a couple of phi1 after the noise
		// timer's, so when both land on the same frame the last bits the
		// noise shifts in arrive too late for the LFO to pick them up
		uint32_t seen = latch ? 16 - s_lfo_noise_lag : 16;
		for (uint32_t step = 0; step < 16; step++)
		{
			if (step == seen)
				m_lfo_noise = m_noise_lfsr;
			step_noise();
		}
	}

	// the noise operator samples the register at a fixed point once per
	// sample; between updates the register is unchanged at that point, so
	// this ends up latching at the noise frequency on its own
	m_noise_state = bitfield(m_noise_lfsr, 1);

	// bit 1 of the test register is officially undocumented but has been
	// discovered to hold the LFO in reset while active; on the chip it forces
	// the phase accumulator input to 0 rather than touching the counters
	if (lfo_reset())
		m_lfo_phase = 0;

	// now pull out the LFO value
	uint32_t lfo = m_lfo_phase;

	// waveform 3 runs the noise word through the same value path the sawtooth
	// uses, so its entry is just the sawtooth entry looked up with a slice of
	// the noise standing in for the phase
	uint32_t rotated = (m_lfo_noise >> s_lfo_noise_shift) |
		(m_lfo_noise << (16 - s_lfo_noise_shift));
	m_lfo_waveform[3][lfo] = m_lfo_waveform[0][bitfield(rotated, 0, 8)];

	// fetch the AM/PM values based on the waveform; AM is unsigned in bits
	// 0-7, while PM is a magnitude in bits 8-15 plus a sign in bit 16
	uint32_t ampm = m_lfo_waveform[lfo_waveform()][lfo];

	// apply depth to the AM value and store for later
	m_lfo_am = (bitfield(ampm, 0, 8) * lfo_am_depth()) >> 7;

	// apply depth to the PM magnitude, then attach the sign
	int32_t pm = (bitfield(ampm, 8, 8) * lfo_pm_depth()) >> 7;
	return bitfield(ampm, 16) ? -pm : pm;
}


//-------------------------------------------------
//  lfo_am_offset - return the AM offset from LFO
//  for the given channel
//-------------------------------------------------

uint32_t opm_registers::lfo_am_offset(uint32_t choffs) const
{
	// OPM maps AM quite differently from OPN

	// shift value for AM sensitivity is [*, 0, 1, 2],
	// mapping to values of [0, 23.9, 47.8, and 95.6dB]
	uint32_t am_sensitivity = ch_lfo_am_sens(choffs);
	if (am_sensitivity == 0)
		return 0;

	// QUESTION: see OPN note below for the dB range mapping; it applies
	// here as well

	// raw LFO AM value on OPM is 0-FF, which is already a factor of 2
	// larger than the OPN below, putting our staring point at 2x theirs;
	// this works out since our minimum is 2x their maximum
	return m_lfo_am << (am_sensitivity - 1);
}


//-------------------------------------------------
//  cache_operator_data - fill the operator cache
//  with prefetched data
//-------------------------------------------------

void opm_registers::cache_operator_data(uint32_t choffs, uint32_t opoffs, opdata_cache &cache)
{
	// set up the easy stuff
	cache.waveform = &m_waveform[0][0];

	// get frequency from the channel
	uint32_t block_freq = cache.block_freq = ch_block_freq(choffs);

	// compute the keycode: block_freq is:
	//
	//     BBBCCCCFFFFFF
	//     ^^^^^
	//
	// the 5-bit keycode is just the top 5 bits (block + top 2 bits
	// of the key code)
	uint32_t keycode = bitfield(block_freq, 8, 5);

	// detune adjustment
	cache.detune = detune_adjustment(op_detune(opoffs), keycode);

	// multiple value, as an x.1 value (0 means 0.5)
	cache.multiple = op_multiple(opoffs) * 2;
	if (cache.multiple == 0)
		cache.multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on
	// block_freq, detune, and multiple, so compute it after we've done those
	if (lfo_pm_depth() == 0 || ch_lfo_pm_sens(choffs) == 0)
		cache.phase_step = compute_phase_step(choffs, opoffs, cache, 0);
	else
		cache.phase_step = opdata_cache::PHASE_STEP_DYNAMIC;

	// total level, scaled by 8
	cache.total_level = op_total_level(opoffs) << 3;

	// 4-bit sustain level, but 15 means 31 so effectively 5 bits
	cache.eg_sustain = op_sustain_level(opoffs);
	cache.eg_sustain |= (cache.eg_sustain + 1) & 0x10;
	cache.eg_sustain <<= 5;

	// determine KSR adjustment for enevlope rates
	uint32_t ksrval = keycode >> (op_ksr(opoffs) ^ 3);
	cache.eg_rate[EG_ATTACK] = effective_rate(op_attack_rate(opoffs) * 2, ksrval);
	cache.eg_rate[EG_DECAY] = effective_rate(op_decay_rate(opoffs) * 2, ksrval);
	cache.eg_rate[EG_SUSTAIN] = effective_rate(op_sustain_rate(opoffs) * 2, ksrval);
	cache.eg_rate[EG_RELEASE] = effective_rate(op_release_rate(opoffs) * 4 + 2, ksrval);
}


//-------------------------------------------------
//  compute_phase_step - compute the phase step
//-------------------------------------------------

uint32_t opm_registers::compute_phase_step(uint32_t choffs, uint32_t opoffs, opdata_cache const &cache, int32_t lfo_raw_pm)
{
	// OPM logic is rather unique here, due to extra detune
	// and the use of key codes (not to be confused with keycode)

	// start with coarse detune delta; table uses cents value from
	// manual, converted into 1/64ths
	static const int16_t s_detune2_delta[4] = { 0, (600*64+50)/100, (781*64+50)/100, (950*64+50)/100 };
	int32_t delta = s_detune2_delta[op_detune2(opoffs)];

	// add in the PM delta
	uint32_t pm_sensitivity = ch_lfo_pm_sens(choffs);
	if (pm_sensitivity != 0)
	{
		// raw PM value is -127..128 which is +/- 200 cents
		// manual gives these magnitudes in cents:
		//    0, +/-5, +/-10, +/-20, +/-50, +/-100, +/-400, +/-700
		// this roughly corresponds to shifting the 200-cent value:
		//    0  >> 5,  >> 4,  >> 3,  >> 2,  >> 1,   << 1,   << 2
		if (pm_sensitivity < 6)
			delta += lfo_raw_pm >> (6 - pm_sensitivity);
		else
			delta += uint32_t(lfo_raw_pm) << (pm_sensitivity - 5);
	}

	// apply delta and convert to a frequency number
	uint32_t phase_step = opm_key_code_to_phase_step(cache.block_freq, delta);

	// apply detune based on the keycode
	phase_step += cache.detune;

	// apply frequency multiplier (which is cached as an x.1 value)
	return (phase_step * cache.multiple) >> 1;
}


//-------------------------------------------------
//  log_keyon - log a key-on event
//-------------------------------------------------

std::string opm_registers::log_keyon(uint32_t choffs, uint32_t opoffs)
{
	uint32_t chnum = choffs;
	uint32_t opnum = opoffs;

	char buffer[256];
	int end = 0;

	end += snprintf(&buffer[end], sizeof(buffer) - end, "%u.%02u freq=%04X dt2=%u dt=%u fb=%u alg=%X mul=%X tl=%02X ksr=%u adsr=%02X/%02X/%02X/%X sl=%X out=%c%c",
		chnum, opnum,
		ch_block_freq(choffs),
		op_detune2(opoffs),
		op_detune(opoffs),
		ch_feedback(choffs),
		ch_algorithm(choffs),
		op_multiple(opoffs),
		op_total_level(opoffs),
		op_ksr(opoffs),
		op_attack_rate(opoffs),
		op_decay_rate(opoffs),
		op_sustain_rate(opoffs),
		op_release_rate(opoffs),
		op_sustain_level(opoffs),
		ch_output_0(choffs) ? 'L' : '-',
		ch_output_1(choffs) ? 'R' : '-');

	bool am = (lfo_am_depth() != 0 && ch_lfo_am_sens(choffs) != 0 && op_lfo_am_enable(opoffs) != 0);
	if (am)
		end += snprintf(&buffer[end], sizeof(buffer) - end, " am=%u/%02X", ch_lfo_am_sens(choffs), lfo_am_depth());
	bool pm = (lfo_pm_depth() != 0 && ch_lfo_pm_sens(choffs) != 0);
	if (pm)
		end += snprintf(&buffer[end], sizeof(buffer) - end, " pm=%u/%02X", ch_lfo_pm_sens(choffs), lfo_pm_depth());
	if (am || pm)
		end += snprintf(&buffer[end], sizeof(buffer) - end, " lfo=%02X/%c", lfo_rate(), "WQTN"[lfo_waveform()]);
	if (noise_enable() && opoffs == 31)
		end += snprintf(&buffer[end], sizeof(buffer) - end, " noise=1");

	return buffer;
}



//*********************************************************
//  YM2151
//*********************************************************

//-------------------------------------------------
//  ym2151 - constructor
//-------------------------------------------------

ym2151::ym2151(ymfm_interface &intf, opm_variant variant) :
	m_variant(variant),
	m_address(0),
	m_fm(intf)
{
}


//-------------------------------------------------
//  reset - reset the system
//-------------------------------------------------

void ym2151::reset()
{
	// reset the engines
	m_fm.reset();
}


//-------------------------------------------------
//  save_restore - save or restore the data
//-------------------------------------------------

void ym2151::save_restore(ymfm_saved_state &state)
{
	m_fm.save_restore(state);
	state.save_restore(m_address);
}


//-------------------------------------------------
//  read_status - read the status register
//-------------------------------------------------

uint8_t ym2151::read_status()
{
	uint8_t result = m_fm.status();
	if (m_fm.intf().ymfm_is_busy())
		result |= fm_engine::STATUS_BUSY;
	return result;
}


//-------------------------------------------------
//  read - handle a read from the device
//-------------------------------------------------

uint8_t ym2151::read(uint32_t offset)
{
	uint8_t result = 0xff;
	switch (offset & 1)
	{
		case 0: // data port (unused)
			debug::log_unexpected_read_write("Unexpected read from YM2151 offset %d\n", offset & 3);
			break;

		case 1: // status port, YM2203 compatible
			result = read_status();
			break;
	}
	return result;
}


//-------------------------------------------------
//  write_address - handle a write to the address
//  register
//-------------------------------------------------

void ym2151::write_address(uint8_t data)
{
	// just set the address
	m_address = data;
}


//-------------------------------------------------
//  write - handle a write to the register
//  interface
//-------------------------------------------------

void ym2151::write_data(uint8_t data)
{
	// write the FM register
	m_fm.write(m_address, data);

	// special cases
	if (m_address == 0x1b)
	{
		// writes to register 0x1B send the upper 2 bits to the output lines
		m_fm.intf().ymfm_external_write(ACCESS_IO, 0, data >> 6);
	}

	// mark busy for a bit
	m_fm.intf().ymfm_set_busy_end(32 * m_fm.clock_prescale());
}


//-------------------------------------------------
//  write - handle a write to the register
//  interface
//-------------------------------------------------

void ym2151::write(uint32_t offset, uint8_t data)
{
	switch (offset & 1)
	{
		case 0: // address port
			write_address(data);
			break;

		case 1: // data port
			write_data(data);
			break;
	}
}


//-------------------------------------------------
//  generate - generate one sample of sound
//-------------------------------------------------

void ym2151::generate(output_data *output, uint32_t numsamples)
{
	for (uint32_t samp = 0; samp < numsamples; samp++, output++)
	{
		// clock the system
		m_fm.clock(fm_engine::ALL_CHANNELS);

		// update the FM content; OPM is full 14-bit with no intermediate clipping
		m_fm.output(output->clear(), 0, 32767, fm_engine::ALL_CHANNELS);

		// YM2151 uses an external DAC (YM3012) with mantissa/exponent format
		// convert to 10.3 floating point value and back to simulate truncation
		output->roundtrip_fp();
	}
}

}
