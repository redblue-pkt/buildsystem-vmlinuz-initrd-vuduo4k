/********************************************
field.c
copyright 1991, Michael D. Brennan

This is a source file for mawk, an implementation of
the AWK programming language.

Mawk is distributed without warranty under the terms of
the GNU General Public License, version 2, 1991.
********************************************/

/*
 * $MawkId: field.c,v 1.13 2010/05/07 22:03:19 tom Exp $
 * @Log: field.c,v @
 * Revision 1.5  1995/06/18  19:17:47  mike
 * Create a type Int which on most machines is an int, but on machines
 * with 16bit ints, i.e., the PC is a long.  This fixes implicit assumption
 * that int==long.
 *
 * Revision 1.4  1994/10/08  19:15:38  mike
 * remove SM_DOS
 *
 * Revision 1.3  1993/07/14  12:32:39  mike
 * run thru indent
 *
 * Revision 1.2	 1993/07/14  12:22:11  mike
 * rm SIZE_T and (void) casts
 *
 * Revision 1.1.1.1  1993/07/03	 18:58:12  mike
 * move source to cvs
 *
 * Revision 5.7	 1993/05/08  18:06:00  mike
 * null_split
 *
 * Revision 5.6	 1993/02/13  21:57:25  mike
 * merge patch3
 *
 * Revision 5.5	 1993/01/01  21:30:48  mike
 * split new_STRING() into new_STRING and new_STRING0
 *
 * Revision 5.4.1.2  1993/01/20	 12:53:08  mike
 * d_to_l()
 *
 * Revision 5.4.1.1  1993/01/15	 03:33:42  mike
 * patch3: safer double to int conversion
 *
 * Revision 5.4	 1992/11/29  22:52:11  mike
 * double->string conversions uses long ints for 16/32 bit
 * compatibility.
 * Fixed small LM_DOS bozo.
 *
 * Revision 5.3	 1992/08/17  14:21:10  brennan
 * patch2: After parsing, only bi_sprintf() uses string_buff.
 *
 * Revision 5.2	 1992/07/10  16:17:10  brennan
 * MsDOS: remove NO_BINMODE macro
 *
 * Revision 5.1	 1991/12/05  07:55:57  brennan
 * 1.1 pre-release
 *
*/

/* field.c */

#include "mawk.h"
#include "field.h"
#include "init.h"
#include "memory.h"
#include "scan.h"
#include "bi_vars.h"
#include "repl.h"
#include "regexp.h"

CELL field[FBANK_SZ + NUM_PFIELDS];

CELL *fbank[NUM_FBANK] =
{field};

static int max_field = MAX_SPLIT;	/* maximum field actually created */

static void build_field0(void);
static void load_field_ov(void);

/* a description of how to split based on RS.
   If RS is changed, so is rs_shadow */
SEPARATOR rs_shadow =
{
    SEP_CHAR, '\n', NULL
};
/* a splitting CELL version of FS */
CELL fs_shadow =
{
    C_SPACE, 0, 0, 0.0
};
int nf;
 /* nf holds the true value of NF.  If nf < 0 , then
    NF has not been computed, i.e., $0 has not been split
  */

static void
set_rs_shadow(void)
{
    CELL c;
    STRING *sval;
    char *s;
    unsigned len;

    if (posix_space_flag && mawk_state == EXECUTION)
	scan_code['\n'] = SC_UNEXPECTED;

    if (rs_shadow.type == SEP_STR) {
	free_STRING((STRING *) rs_shadow.ptr);
    }

    cast_for_split(cellcpy(&c, RS));
    switch (c.type) {
    case C_RE:
	if ((s = is_string_split(c.ptr, &len))) {
	    if (len == 1) {
		rs_shadow.type = SEP_CHAR;
		rs_shadow.c = s[0];
	    } else {
		rs_shadow.type = SEP_STR;
		rs_shadow.ptr = (PTR) new_STRING(s);
	    }
	} else {
	    rs_shadow.type = SEP_RE;
	    rs_shadow.ptr = c.ptr;
	}
	break;

    case C_SPACE:
	rs_shadow.type = SEP_CHAR;
	rs_shadow.c = ' ';
	break;

    case C_SNULL:		/* RS becomes one or more blank lines */
	if (mawk_state == EXECUTION)
	    scan_code['\n'] = SC_SPACE;
	rs_shadow.type = SEP_MLR;
	sval = new_STRING("\n\n+");
	rs_shadow.ptr = re_compile(sval);
	free_STRING(sval);
	break;

    case C_STRING:
	/*
	 * Check for special case where we retained the cell as a string,
	 * bypassing regular-expression compiling.
	 */
	if (string(&c)->len == 1) {
	    rs_shadow.type = SEP_CHAR;
	    rs_shadow.c = string(&c)->str[0];
	    break;
	}
	/* FALLTHRU */
    default:
	bozo("bad cell in set_rs_shadow");
    }
}

static void
load_pfield(const char *name, CELL * cp)
{
    SYMTAB *stp;

    stp = insert(name);
    stp->type = ST_FIELD;
    stp->stval.cp = cp;
}

/* initialize $0 and the pseudo fields */
void
field_init(void)
{
    field[0].type = C_STRING;
    field[0].ptr = (PTR) & null_str;
    null_str.ref_cnt++;

    load_pfield("NF", NF);
    NF->type = C_DOUBLE;
    NF->dval = 0.0;

    load_pfield("RS", RS);
    RS->type = C_STRING;
    RS->ptr = (PTR) new_STRING("\n");
    /* rs_shadow already set */

    load_pfield("FS", FS);
    FS->type = C_STRING;
    FS->ptr = (PTR) new_STRING(" ");
    /* fs_shadow is already set */

    load_pfield("OFMT", OFMT);
    OFMT->type = C_STRING;
    OFMT->ptr = (PTR) new_STRING("%.6g");

    load_pfield("CONVFMT", CONVFMT);
    CONVFMT->type = C_STRING;
    CONVFMT->ptr = OFMT->ptr;
    string(OFMT)->ref_cnt++;
}

void
set_field0(char *s, size_t len)
{
    cell_destroy(&field[0]);
    nf = -1;

    if (len) {
	field[0].type = C_MBSTRN;
	field[0].ptr = (PTR) new_STRING0(len);
	memcpy(string(&field[0])->str, s, len);
    } else {
	field[0].type = C_STRING;
	field[0].ptr = (PTR) & null_str;
	null_str.ref_cnt++;
    }
}

/* split field[0] into $1, $2 ... and set NF  */

void
split_field0(void)
{
    register CELL *cp;
    register int cnt;
    CELL c;			/* copy field[0] here if not string */

    if (field[0].type < C_STRING) {
	cast1_to_s(cellcpy(&c, field + 0));
	cp = &c;
    } else
	cp = &field[0];

    if (string(cp)->len == 0)
	nf = 0;
    else {
	switch (fs_shadow.type) {
	case C_SNULL:		/* FS == "" */
	    nf = (int) null_split(string(cp)->str, string(cp)->len);
	    break;

	case C_SPACE:
	    nf = (int) space_split(string(cp)->str, string(cp)->len);
	    break;

	default:
	    nf = (int) re_split(string(cp), fs_shadow.ptr);
	    break;
	}

    }

    cell_destroy(NF);
    NF->type = C_DOUBLE;
    NF->dval = (double) nf;

    if (nf > MAX_SPLIT) {
	cnt = MAX_SPLIT;
	load_field_ov();
    } else
	cnt = nf;

    while (cnt > 0) {
	cell_destroy(field + cnt);
	field[cnt].ptr = (PTR) split_buff[cnt - 1];
	field[cnt--].type = C_MBSTRN;
    }

    if (cp == &c) {
	free_STRING(string(cp));
    }
}

/*
  assign CELL *cp to field or pseudo field
  and take care of all side effects
*/

void
field_assign(CELL * fp, CELL * cp)
{
    CELL c;
    int i, j;

    /* the most common case first */
    if (fp == field) {
	cell_destroy(field);
	cellcpy(fp, cp);
	nf = -1;
	return;
    }

    /* its not important to do any of this fast */

    if (nf < 0)
	split_field0();

#ifdef  MSDOS
    if (!SAMESEG(fp, field)) {
	i = -1;
	goto lm_dos_label;
    }
#endif

    switch (i = (int) (fp - field)) {

    case NF_field:

	cell_destroy(NF);
	cellcpy(NF, cellcpy(&c, cp));
	if (c.type != C_DOUBLE)
	    cast1_to_d(&c);

	if ((j = d_to_i(c.dval)) < 0)
	    rt_error("negative value assigned to NF");

	if (j > nf)
	    for (i = nf + 1; i <= j; i++) {
		cp = field_ptr(i);
		cell_destroy(cp);
		cp->type = C_STRING;
		cp->ptr = (PTR) & null_str;
		null_str.ref_cnt++;
	    }

	nf = j;
	build_field0();
	break;

    case RS_field:
	cell_destroy(RS);
	cellcpy(RS, cp);
	set_rs_shadow();
	break;

    case FS_field:
	cell_destroy(FS);
	cast_for_split(cellcpy(&fs_shadow, cellcpy(FS, cp)));
	break;

    case OFMT_field:
    case CONVFMT_field:
	/* If the user does something stupid with OFMT or CONVFMT,
	   we could crash.
	   We'll make an attempt to protect ourselves here.  This is
	   why OFMT and CONVFMT are pseudo fields.

	   The ptrs of OFMT and CONVFMT always have a valid STRING,
	   even if assigned a DOUBLE or NOINIT
	 */

	free_STRING(string(fp));
	cellcpy(fp, cp);
	if (fp->type < C_STRING)	/* !! */
	    fp->ptr = (PTR) new_STRING("%.6g");
	else if (fp == CONVFMT) {
	    /* It's a string, but if it's really goofy and CONVFMT,
	       it could still damage us. Test it .
	     */
	    char xbuff[512];

	    xbuff[256] = 0;
	    sprintf(xbuff, string(fp)->str, 3.1459);
	    if (xbuff[256])
		rt_error("CONVFMT assigned unusable value");
	}
	break;

#ifdef MSDOS
      lm_dos_label:
#endif

    default:			/* $1 or $2 or ... */

	cell_destroy(fp);
	cellcpy(fp, cp);

	if (i < 0 || i > MAX_SPLIT)
	    i = field_addr_to_index(fp);

	if (i > nf) {
	    for (j = nf + 1; j < i; j++) {
		cp = field_ptr(j);
		cell_destroy(cp);
		cp->type = C_STRING;
		cp->ptr = (PTR) & null_str;
		null_str.ref_cnt++;
	    }
	    nf = i;
	    cell_destroy(NF);
	    NF->type = C_DOUBLE;
	    NF->dval = (double) i;
	}

	build_field0();

    }
}

/* construct field[0] from the other fields */

static void
build_field0(void)
{

#ifdef DEBUG
    if (nf < 0)
	bozo("nf <0 in build_field0");
#endif

    cell_destroy(field + 0);

    if (nf == 0) {
	field[0].type = C_STRING;
	field[0].ptr = (PTR) & null_str;
	null_str.ref_cnt++;
    } else if (nf == 1) {
	cellcpy(field, field + 1);
    } else {
	CELL c;
	STRING *ofs, *tail;
	size_t len;
	register CELL *cp;
	register char *p, *q;
	int cnt;
	CELL **fbp, *cp_limit;

	cast1_to_s(cellcpy(&c, OFS));
	ofs = (STRING *) c.ptr;
	cast1_to_s(cellcpy(&c, field_ptr(nf)));
	tail = (STRING *) c.ptr;
	cnt = nf - 1;

	len = ((size_t) cnt) * ofs->len + tail->len;

	fbp = fbank;
	cp_limit = field + FBANK_SZ;
	cp = field + 1;

	while (cnt-- > 0) {
	    if (cp->type < C_STRING) {	/* use the string field temporarily */
		if (cp->type == C_NOINIT) {
		    cp->ptr = (PTR) & null_str;
		    null_str.ref_cnt++;
		} else {	/* its a double */
		    Int ival;
		    char xbuff[260];

		    ival = d_to_I(cp->dval);
		    if (ival == cp->dval)
			sprintf(xbuff, INT_FMT, ival);
		    else
			sprintf(xbuff, string(CONVFMT)->str, cp->dval);

		    cp->ptr = (PTR) new_STRING(xbuff);
		}
	    }

	    len += string(cp)->len;

	    if (++cp == cp_limit) {
		cp = *++fbp;
		cp_limit = cp + FBANK_SZ;
	    }

	}

	field[0].type = C_STRING;
	field[0].ptr = (PTR) new_STRING0(len);

	p = string(field)->str;

	/* walk it again , putting things together */
	cnt = nf - 1;
	fbp = fbank;
	cp = field + 1;
	cp_limit = field + FBANK_SZ;
	while (cnt-- > 0) {
	    memcpy(p, string(cp)->str, string(cp)->len);
	    p += string(cp)->len;
	    /* if not really string, free temp use of ptr */
	    if (cp->type < C_STRING) {
		free_STRING(string(cp));
	    }
	    if (++cp == cp_limit) {
		cp = *++fbp;
		cp_limit = cp + FBANK_SZ;
	    }
	    /* add the separator */
	    q = ofs->str;
	    while (*q)
		*p++ = *q++;
	}
	/* tack tail on the end */
	memcpy(p, tail->str, tail->len);

	/* cleanup */
	free_STRING(tail);
	free_STRING(ofs);
    }
}

/* We are assigning to a CELL and we aren't sure if its
   a field */

void
slow_cell_assign(CELL * target, CELL * source)
{
    if (

#ifdef MSDOS			/* the dreaded segment nonsense */
	   SAMESEG(target, field) &&
#endif
	   target >= field && target <= LAST_PFIELD)
	field_assign(target, source);
    else {
	CELL **p = fbank + 1;

	while (*p) {
	    if (
#ifdef  MSDOS
		   SAMESEG(target, *p) &&
#endif
		   target >= *p && target < *p + FBANK_SZ) {
		field_assign(target, source);
		return;
	    }
	    p++;
	}
	/* its not a field */
	cell_destroy(target);
	cellcpy(target, source);
    }
}

int
field_addr_to_index(CELL * cp)
{
    CELL **p = fbank;

    while (

#ifdef MSDOS
	      !SAMESEG(cp, *p) ||
#endif

	      cp < *p || cp >= *p + FBANK_SZ)
	p++;

    return (int) (((p - fbank) << FB_SHIFT) + (cp - *p));
}

/*------- more than 1 fbank needed  ------------*/

/*
  compute the address of a field with index
  > MAX_SPLIT
*/

CELL *
slow_field_ptr(int i)
{

    if (i > max_field) {
	int j;

	if (i > MAX_FIELD)
	    rt_overflow("maximum number of fields", MAX_FIELD);

	j = 1;
	while (fbank[j])
	    j++;

	do {
	    fbank[j] = (CELL *) zmalloc(sizeof(CELL) * FBANK_SZ);
	    memset(fbank[j], 0, sizeof(CELL) * FBANK_SZ);
	    j++;
	    max_field += FBANK_SZ;
	}
	while (i > max_field);
    }

    return &fbank[i >> FB_SHIFT][i & (FBANK_SZ - 1)];
}

/*
  $0 split into more than MAX_SPLIT fields,
  $(MAX_FIELD+1) ... are on the split_ov_list.
  Copy into fields which start at fbank[1]
*/

static void
load_field_ov(void)
{
    register SPLIT_OV *p;	/* walks split_ov_list */
    register CELL *cp;		/* target of copy */
    int j;			/* current fbank[] */
    CELL *cp_limit;		/* change fbank[] */
    SPLIT_OV *q;		/* trails p */

    /* make sure the fields are allocated */
    slow_field_ptr(nf);

    p = split_ov_list;
    split_ov_list = (SPLIT_OV *) 0;
    j = 1;
    cp = fbank[j];
    cp_limit = cp + FBANK_SZ;
    while (p) {
	cell_destroy(cp);
	cp->type = C_MBSTRN;
	cp->ptr = (PTR) p->sval;

	if (++cp == cp_limit) {
	    cp = fbank[++j];
	    cp_limit = cp + FBANK_SZ;
	}

	q = p;
	p = p->link;
	ZFREE(q);
    }
}

#if USE_BINMODE

/* read current value of BINMODE */
int
binmode(void)
{
    CELL c;

    cast1_to_d(cellcpy(&c, BINMODE));
    return d_to_i(c.dval);
}

/* set BINMODE and RS and ORS
   from environment or -W binmode=   */

void
set_binmode(int x)
{
    CELL c;
    int change = ((x & 4) == 0);

    /* set RS */
    c.type = C_STRING;
    c.ptr = (PTR) new_STRING((change && (x & 1)) ? "\r\n" : "\n");
    field_assign(RS, &c);
    free_STRING(string(&c));

    /* set ORS */
    cell_destroy(ORS);
    ORS->type = C_STRING;
    ORS->ptr = (PTR) new_STRING((change && (x & 2)) ? "\r\n" : "\n");

    cell_destroy(BINMODE);
    BINMODE->type = C_DOUBLE;
    BINMODE->dval = (double) x;
}

#endif /* USE_BINMODE */
