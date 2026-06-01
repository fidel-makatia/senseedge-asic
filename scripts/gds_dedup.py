#!/usr/bin/env python3
"""
Fix Magic-streamout GDS where some cells have a DUPLICATE BGNSTR+STRNAME
prefix before their actual content. Removes the extra prefix while keeping
the cell content intact.

Malformed:
  BGNSTR(date) | STRNAME(X) | BGNSTR(date) | STRNAME(X) | BOUNDARY... | ENDSTR
Fixed:
  BGNSTR(date) | STRNAME(X) | BOUNDARY... | ENDSTR
"""
import struct
import sys

REC_BGNSTR = 0x05
REC_STRNAME = 0x06
REC_ENDSTR = 0x07
REC_ENDLIB = 0x04


def read_records(buf):
    i = 0
    n = len(buf)
    while i + 4 <= n:
        size = struct.unpack(">H", buf[i:i+2])[0]
        rtype = buf[i+2]
        if size < 4:
            raise ValueError(f"bad record size {size} at offset {i}")
        end = i + size
        yield i, rtype, buf[i:end]
        i = end


def fix(in_path, out_path):
    with open(in_path, "rb") as f:
        buf = f.read()

    out = bytearray()
    in_struct = False        # are we inside a BGNSTR..ENDSTR?
    seen_strname = False     # already saw STRNAME for current cell?
    pending_bgnstr = None    # an unmatched BGNSTR we're holding
    n_fixed = 0
    n_cells = 0

    for off, rtype, rec in read_records(buf):
        if not in_struct:
            if rtype == REC_BGNSTR:
                in_struct = True
                seen_strname = False
                pending_bgnstr = None
                out += rec       # emit the BGNSTR
            else:
                out += rec
                if rtype == REC_ENDLIB:
                    break
        else:
            # inside a structure
            if rtype == REC_BGNSTR:
                # Malformed: a 2nd BGNSTR before ENDSTR.
                # Hold it; we'll emit it only if it's not followed by a
                # duplicate STRNAME (i.e. it really starts a new structure
                # with no ENDSTR — unlikely but defensive).
                pending_bgnstr = rec
                continue
            if rtype == REC_STRNAME:
                if seen_strname:
                    # Duplicate STRNAME — drop the held BGNSTR and this STRNAME.
                    pending_bgnstr = None
                    n_fixed += 1
                    print(f"  fixed duplicate header at offset {off}: {rec[4:].rstrip(chr(0).encode()).decode('latin-1')}", file=sys.stderr)
                    continue
                else:
                    seen_strname = True
                    if pending_bgnstr is not None:
                        out += pending_bgnstr
                        pending_bgnstr = None
                    out += rec
                    continue
            # any other record inside structure
            if pending_bgnstr is not None:
                # The pending BGNSTR was followed by a non-STRNAME — flush it.
                out += pending_bgnstr
                pending_bgnstr = None
            out += rec
            if rtype == REC_ENDSTR:
                in_struct = False
                n_cells += 1
                seen_strname = False

    with open(out_path, "wb") as f:
        f.write(bytes(out))

    print(f"cells emitted: {n_cells}", file=sys.stderr)
    print(f"duplicate headers fixed: {n_fixed}", file=sys.stderr)
    print(f"input  {in_path}: {len(buf):,} bytes", file=sys.stderr)
    print(f"output {out_path}: {len(out):,} bytes", file=sys.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <input.gds> <output.gds>", file=sys.stderr)
        sys.exit(1)
    fix(sys.argv[1], sys.argv[2])
