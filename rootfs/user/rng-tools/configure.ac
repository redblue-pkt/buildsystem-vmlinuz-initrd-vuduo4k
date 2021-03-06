dnl Process this file with autoconf 2.52+ to produce a configure script.
dnl
dnl Copyright (C) 2001 Philipp Rumpf
dnl Copyright (C) 2004 Henrique de Moraes Holschuh <hmh@debian.org>
dnl
dnl This program is free software; you can redistribute it and/or modify
dnl it under the terms of the GNU General Public License as published by
dnl the Free Software Foundation; either version 2 of the License, or
dnl (at your option) any later version.
dnl
dnl This program is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY; without even the implied warranty of
dnl MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
dnl GNU General Public License for more details.
dnl
dnl You should have received a copy of the GNU General Public License
dnl along with this program; if not, write to the Free Software
dnl Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335  USA

AC_INIT(rng-tools, 5, [Jeff Garzik <jgarzik@pobox.com>])
AC_PREREQ(2.52)
AC_CONFIG_SRCDIR([rngd.c])
AM_INIT_AUTOMAKE([gnu])
AC_CONFIG_HEADERS([rng-tools-config.h])

dnl Parse options

AC_ARG_WITH([libgcrypt],
	AS_HELP_STRING([--without-libgcrypt],
		[Disable libgcrypt support. Systems that support RDRAND but not AES-NI will require libgcrypt in order to use RDRAND as an entropy source. (Default: --with-libgcrypt)]),
	[],
	[with_libgcrypt=check]
)

dnl Make sure anyone changing configure.ac/Makefile.am has a clue
AM_MAINTAINER_MODE

dnl Checks for programs
AC_PROG_CC
AC_PROG_RANLIB
AC_PROG_GCC_TRADITIONAL

dnl Checks for header files.
dnl AC_HEADER_STDC
dnl AC_CHECK_HEADERS(sys/ioctl.h unistd.h)

dnl Checks for typedefs, structures, and compiler characteristics.
dnl AC_TYPE_SIZE_T
dnl AC_TYPE_PID_T

dnl -----------------------------
dnl Checks for required libraries
dnl -----------------------------

dnl -------------------------------------
dnl Checks for optional library functions
dnl -------------------------------------

dnl -------------------------------------
dnl Check for libgcrypt support
dnl -------------------------------------

AS_IF(
	[test "x$with_libgcrypt" != "xno"],
	[
		AC_CHECK_HEADER([gcrypt.h],
			AC_CHECK_LIB(
				[gcrypt],
				[gcry_check_version], ,
				[
					if test "x$with_libgcrypt" != "xcheck"; then
						AC_MSG_FAILURE([libgcrypt not found]); else
						AC_MSG_NOTICE([libgcrypt support disabled])
					fi
				]
			),
			[if test "x$with_libgcrypt" != "xcheck"; then
				AC_MSG_FAILURE([libgcrypt headers not found]); else
				AC_MSG_NOTICE([libgcrypt support disabled])
			fi]
		)
	]
)

dnl -----------------
dnl Configure options
dnl -----------------

AM_PROG_AS

dnl --------------------------
dnl autoconf output generation
dnl --------------------------

AC_CONFIG_FILES([Makefile contrib/Makefile rngd.8 rngtest.1])
AC_OUTPUT
