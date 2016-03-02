/* -*- c++ -*- */
/*
 * Copyright 2005,2010 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/mman.h>

#include "source_f_impl.h"
#include <gnuradio/io_signature.h>
#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <stdexcept>

// FIXME these should query some kind of user preference

namespace gr {
  namespace comedi {

    static comedi_range * range_info[256];
    static lsampl_t maxdata[256];

    static std::string
    default_device_name()
    {
      return "/dev/comedi0";
    }

    // ----------------------------------------------------------------

    source_f::sptr
    source_f::make(int sampling_freq, const std::string dev)
    {
      return gnuradio::get_initial_sptr
	(new source_f_impl(sampling_freq, dev));
    }

    source_f_impl::source_f_impl(int sampling_freq,
				 const std::string device_name)
      : sync_block("comedi_source_f",
		      io_signature::make(0, 0, 0),
		      io_signature::make(0, 0, 0)),
	d_sampling_freq(sampling_freq),
	d_device_name(device_name.empty() ? default_device_name() : device_name),
	d_dev(0),
	d_subdevice(0/*COMEDI_SUBD_AI*/),
	d_n_chan(1),	// number of channels to read
	d_map(0),
	d_buffer_size(0),
	d_buf_front(0),
	d_buf_back(0),
	d_sample_size(0)
    {
      int aref = AREF_GROUND;
      int range = 0;

      d_dev = comedi_open(d_device_name.c_str());
      if(d_dev == 0) {
	comedi_perror(d_device_name.c_str());
	throw std::runtime_error("source_f_impl");
      }

      comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);

      unsigned int chanlist[256];

      for(int i=0; i<d_n_chan; i++) {
	chanlist[i] = CR_PACK(i,range,aref);
	range_info[i] = comedi_get_range(d_dev, d_subdevice, 0, range);
	maxdata[i] = comedi_get_maxdata(d_dev, d_subdevice, 0);
      }

      comedi_cmd cmd;
      int ret;

      ret = comedi_get_cmd_generic_timed(d_dev, d_subdevice,
					 &cmd, d_n_chan,
					 (unsigned int)(1e9/sampling_freq));
      if(ret < 0)
	bail("comedi_get_cmd_generic_timed", comedi_errno());

      // TODO: check period_ns is not to far off sampling_freq

      d_buffer_size = comedi_get_buffer_size(d_dev, d_subdevice);
      if(d_buffer_size <= 0)
	bail("comedi_get_buffer_size", comedi_errno());

      d_map = mmap(NULL, d_buffer_size, PROT_READ,
		   MAP_SHARED, comedi_fileno(d_dev),0);
      if(d_map == MAP_FAILED)
	bail("mmap", errno);

      ret = comedi_get_subdevice_flags(d_dev, d_subdevice);
      if(ret < 0){
	bail("comedi_get_subdevice_flags", comedi_errno());
      }
      d_sample_size = (ret & SDF_LSAMPL)
		? sizeof(lsampl_t) : sizeof(sampl_t);
      printf("comedi device sample size is %u\n", d_sample_size);
      printf("comedi device channel number is %u\n", d_n_chan);

      cmd.chanlist = chanlist;
      cmd.chanlist_len = d_n_chan;
      cmd.scan_end_arg = d_n_chan;
      cmd.stop_src = TRIG_NONE;
      cmd.stop_arg = 0;

      /* comedi_command_test() tests a command to see if the trigger
       * sources and arguments are valid for the subdevice.  If a
       * trigger source is invalid, it will be logically ANDed with
       * valid values (trigger sources are actually bitmasks), which
       * may or may not result in a valid trigger source.  If an
       * argument is invalid, it will be adjusted to the nearest valid
       * value.  In this way, for many commands, you can test it
       * multiple times until it passes.  Typically, if you can't get
       * a valid command in two tests, the original command wasn't
       * specified very well. */
      ret = comedi_command_test(d_dev,&cmd);

      if(ret < 0)
	bail("comedi_command_test", comedi_errno());

      ret = comedi_command_test(d_dev,&cmd);

      if(ret < 0)
	bail("comedi_command_test", comedi_errno());

      /* start the command */
      ret = comedi_command(d_dev,&cmd);

      if(ret < 0)
	bail("comedi_command", comedi_errno());

      set_output_multiple(d_n_chan*sizeof(float));

      set_output_signature(io_signature::make(1, 1, sizeof(float)));
    }

    bool
    source_f_impl::check_topology(int ninputs, int noutputs)
    {
      if(noutputs > d_n_chan)
	throw std::runtime_error("source_f_impl");

      return true;
    }

    source_f_impl::~source_f_impl()
    {
      if(d_map) {
	munmap(d_map, d_buffer_size);
	d_map = 0;
      }

      comedi_close(d_dev);
    }

    int
    source_f_impl::work(int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items)
    {
      int ret;

      int work_left = noutput_items * d_sample_size * d_n_chan;

      do {
	do {
	  ret = comedi_get_buffer_contents(d_dev, d_subdevice);
	  if(ret < 0)
	    bail("comedi_get_buffer_contents", comedi_errno());
	  printf("noutput_items=%i, ret=%i, work_left=%i, front=%i, back=%i \n", noutput_items, ret, work_left, d_buf_front, d_buf_back);


	  assert(ret % d_sample_size == 0);
	  assert(work_left % d_sample_size == 0);

	  ret = std::min(ret, work_left);
	  d_buf_front += ret;

	  assert(d_buffer_size%d_n_chan == 0);
	  if(d_buf_front-d_buf_back > (unsigned)d_buffer_size) {
	    d_buf_front+=d_buffer_size;
	    d_buf_back +=d_buffer_size;
	  }

	  if(d_buf_front==d_buf_back) {
	    usleep(1000000*std::min(work_left,
				    d_buffer_size/2)/(d_sampling_freq*d_sample_size*d_n_chan));	
	    continue;
	  }
	} while(d_buf_front == d_buf_back);

	
	for(unsigned i=d_buf_back/d_sample_size; i < d_buf_front/d_sample_size; i++) {
	  int chan = i%d_n_chan;
	  int o_idx = noutput_items-work_left/d_n_chan/d_sample_size + \
	    (i-d_buf_back/d_sample_size)/d_n_chan;

	  if(output_items[chan])
	    if (d_sample_size == sizeof(lsampl_t)) {
	    ((float*)(output_items[chan]))[o_idx] =
	      (float)comedi_to_phys(*(((lsampl_t*)d_map)+i%(d_buffer_size/d_sample_size)), range_info[chan], maxdata[chan]);
            } ;
            if (d_sample_size == sizeof(sampl_t)) {
	    ((float*)(output_items[chan]))[o_idx] =
	      (float)comedi_to_phys(*(((sampl_t*)d_map)+i%(d_buffer_size/d_sample_size)), range_info[chan], maxdata[chan]);
            } ;  
	}

	ret = comedi_mark_buffer_read(d_dev,d_subdevice, d_buf_front-d_buf_back);
	if(ret < 0)
	  bail("comedi_mark_buffer_read", comedi_errno());

	work_left -= d_buf_front-d_buf_back;

	d_buf_back = d_buf_front;
      } while(work_left > 0);

      return noutput_items;
    }

    void
    source_f_impl::output_error_msg(const char *msg, int err)
    {
      fprintf(stderr, "source_f_impl[%s]: %s: %s\n",
	      d_device_name.c_str(), msg,  comedi_strerror(err));
    }

    void
    source_f_impl::bail(const char *msg, int err) throw (std::runtime_error)
    {
      output_error_msg(msg, err);
      throw std::runtime_error("source_f_impl");
    }

  } /* namespace comedi */
} /* namespace gr */
