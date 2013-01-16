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
 *
 * $FreeBSD$
 */

/*
 * DIFFUSE packet length feature.
 */

#ifndef _NETINET_IPFW_DIFFUSE_FEATURE_PLEN_H_
#define _NETINET_IPFW_DIFFUSE_FEATURE_PLEN_H_

#define	DI_DEFAULT_PLEN_WINDOW 15

/* Main properties, used in kernel and userspace. */
#define	DI_PLEN_NAME "plen"
#define	DI_PLEN_TYPE DI_FEATURE_ALG_UNIDIRECTIONAL
#define	DI_PLEN_NO_STATS 5
#define	DI_PLEN_STAT_NAMES_DECL char *di_plen_stat_names[DI_PLEN_NO_STATS]
#define	DI_PLEN_STAT_NAMES DI_PLEN_STAT_NAMES_DECL =			\
{									\
	"min",								\
	"mean",								\
	"max",								\
	"stdev",							\
	"sum"								\
};

struct di_feature_module * plen_module(void);

#endif /* _NETINET_IPFW_DIFFUSE_FEATURE_PLEN_H_ */