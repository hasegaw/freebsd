/*-
 * Copyright (c) 2010-2011
 * 	Swinburne University of Technology, Melbourne, Australia.
 * All rights reserved.
 *
 * This software was developed at the Centre for Advanced Internet
 * Architectures, Swinburne University of Technology, by Sebastian Zander, made
 * possible in part by a gift from The Cisco University Research Program Fund, a
 * corporate advised fund of Silicon Valley Community Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Description:
 * Packet count feature.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/queue.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <net/if.h>

#include <netinet/in.h>
#include <netinet/ip_fw.h>
#include <netinet/ip_diffuse.h>

#include <netinet/ipfw/diffuse_feature.h>
#include <netinet/ipfw/diffuse_feature_pcnt.h>

#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <libutil.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysexits.h>

#include "diffuse_ui.h"
#include "ipfw2.h"

enum feature_pcnt_tokens {
	TOK_DI_WINDOW = TOK_DI_FEATURE_MOD_START,
	TOK_DI_PARTIAL_WINDOWS,
	TOK_DI_JUMP_WINDOWS
};

static struct di_option feature_pcnt_params[] = {
	{ "window",		DI_OPTION_ARG_UINT,	1,	1000,
	    TOK_DI_WINDOW },
	{ "partial-windows",	DI_OPTION_ARG_NOARG,	0,	0,
	    TOK_DI_PARTIAL_WINDOWS },
	{ "jump-windows",	DI_OPTION_ARG_NOARG,	0,	0,
	    TOK_DI_JUMP_WINDOWS },
	{ NULL, 0, 0 }  /* Terminator. */
};

int
pcnt_get_conf_size(void)
{

	return (sizeof(struct di_feature_pcnt_config));
}

int
pcnt_get_opts(struct di_option **opts)
{

	*opts = feature_pcnt_params;

	return (sizeof(feature_pcnt_params));
}

int
pcnt_parse_opts(int token, char *arg_val, struct di_oid *buf)
{
	static struct di_feature_pcnt_config *conf = NULL;
	char *end;

	if (conf == NULL) {
		conf = (struct di_feature_pcnt_config *)buf;
		conf->pcnt_window = -1;
		conf->pcnt_partial_window = -1;
		conf->pcnt_jump_window = -1;
	}

	switch(token) {
	case TOK_DI_OPTS_INIT:
		break;

	case TOK_DI_WINDOW:
		end = NULL;
		conf->pcnt_window = strtoul(arg_val, &end, 0);
		if (*end == 'K' || *end == 'k')
			conf->pcnt_window *= 1024;
		break;

	case TOK_DI_PARTIAL_WINDOWS:
		conf->pcnt_partial_window = 1;
		break;

	case TOK_DI_JUMP_WINDOWS:
		conf->pcnt_jump_window = 1;
		break;

	default:
		/* This should never happen. */
		errx(EX_DATAERR, "invalid option, fix source");
	}

	return (0);
}

void
pcnt_print_opts(struct di_oid *opts)
{
	struct di_feature_pcnt_config *conf;

	conf = (struct di_feature_pcnt_config *)opts;

	printf("  window: %d\n", conf->pcnt_window);
	printf("  partial windows: %s\n",
	    (conf->pcnt_partial_window == 1) ? "yes" : "no");
	printf("  jump window: %s\n",
	    (conf->pcnt_jump_window == 1) ? "yes" : "no");
}

void
pcnt_print_usage()
{

	printf("module pcnt [window <packets>] [partial-windows] "
	    "[jump-windows]\n");
}

DI_PCNT_STAT_NAMES; /* Stat name array in diffuse_feature_pcnt.h. */
char *
pcnt_get_stat_name(int i)
{

	return (di_pcnt_stat_names[i]);
}

static struct di_feature_module pcnt_feature_module = {
	.name =			DI_PCNT_NAME,
	.type =			DI_PCNT_TYPE,
	.get_conf_size =	pcnt_get_conf_size,
	.get_opts =		pcnt_get_opts,
	.parse_opts =		pcnt_parse_opts,
	.print_opts =		pcnt_print_opts,
	.print_usage =		pcnt_print_usage,
	.get_stat_name =	pcnt_get_stat_name
};

struct di_feature_module *
pcnt_module(void)
{

	return (&pcnt_feature_module);
}