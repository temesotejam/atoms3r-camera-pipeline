#!/usr/bin/env python3
"""Exercise the proof verifier on small real synthetic ELF32 files."""
import io
import struct
import unittest
from elftools.elf.elffile import ELFFile
from verify_angle_flag_symbol import verify, EXPECTED


def fixture(payload=EXPECTED, symbol_name=b'angle_trial_build_flag_proof', symbol_size=None,
            duplicate=False, bind_type=0x11):
    shnames=b'\0.shstrtab\0.strtab\0.symtab\0.proof\0'
    strings=b'\0'+symbol_name+b'\0'
    sym=bytes(16)+struct.pack('<IIIBBH',1,0x1000,len(payload) if symbol_size is None else symbol_size,bind_type,0,4)
    if duplicate: sym+=sym[16:32]
    # Prefix printable data to reproduce a strings(1) line with an adjacent
    # printable prefix while the symbol itself still holds the exact bytes.
    proof=b'PREFIX'+payload
    sym=bytearray(sym);struct.pack_into('<I',sym,16+4,0x1006)
    if duplicate: struct.pack_into('<I',sym,32+4,0x1006)
    chunks=[shnames,strings,bytes(sym),proof]
    offsets=[];cursor=52+40*5
    for b in chunks:offsets.append(cursor);cursor+=len(b)
    hdr=struct.pack('<16sHHIIIIIHHHHHH',b'\x7fELF\x01\x01\x01'+bytes(9),1,94,1,0,0,52,0,52,0,0,40,5,1)
    def section(name,kind,flags,addr,offset,size,link=0,info=0,align=1,entsize=0):
        return struct.pack('<IIIIIIIIII',name,kind,flags,addr,offset,size,link,info,align,entsize)
    sec=bytes(40)+section(shnames.index(b'.shstrtab'),3,0,0,offsets[0],len(chunks[0]))
    sec+=section(shnames.index(b'.strtab'),3,0,0,offsets[1],len(chunks[1]))
    sec+=section(shnames.index(b'.symtab'),2,0,0,offsets[2],len(chunks[2]),2,1,4,16)
    sec+=section(shnames.index(b'.proof'),1,2,0x1000,offsets[3],len(chunks[3]))
    return ELFFile(io.BytesIO(hdr+sec+b''.join(chunks)))


class TestProof(unittest.TestCase):
    def test_exact_symbol_even_with_adjacent_printable_prefix(self):
        verify(fixture())
    def test_tampered_value(self):
        with self.assertRaisesRegex(ValueError, 'bytes differ'):
            verify(fixture(b'angle-trial-flags:1,0,0\0'))
    def test_missing(self):
        with self.assertRaisesRegex(ValueError, 'exactly once'):
            verify(fixture(symbol_name=b'other'))
    def test_duplicate(self):
        with self.assertRaisesRegex(ValueError, 'exactly once'):
            verify(fixture(duplicate=True))
    def test_size_and_binding(self):
        for elf in (fixture(symbol_size=1),fixture(bind_type=0x01)):
            with self.assertRaises(ValueError):verify(elf)


if __name__ == '__main__':unittest.main()
