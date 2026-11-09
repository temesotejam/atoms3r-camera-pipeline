#!/usr/bin/env python3
"""Verify the exact bytes of the linked trial flag proof symbol."""
import argparse
from elftools.elf.elffile import ELFFile

SYMBOL = 'angle_trial_build_flag_proof'
EXPECTED = b'angle-trial-flags:1,1,0\0'


def verify(elf):
    symtab = elf.get_section_by_name('.symtab')
    if symtab is None or symtab['sh_type'] != 'SHT_SYMTAB':
        raise ValueError('ELF has no static symbol table')
    found = symtab.get_symbol_by_name(SYMBOL) or []
    if len(found) != 1:
        raise ValueError('trial flag proof must be present exactly once')
    symbol = found[0]
    if symbol['st_info']['bind'] != 'STB_GLOBAL' or symbol['st_info']['type'] != 'STT_OBJECT':
        raise ValueError('trial flag proof is not a global object')
    if symbol['st_size'] != len(EXPECTED):
        raise ValueError('trial flag proof size differs')
    index = symbol['st_shndx']
    if not isinstance(index, int) or index <= 0 or index >= elf.num_sections():
        raise ValueError('trial flag proof has no defined section')
    section = elf.get_section(index)
    if section is None or section['sh_type'] != 'SHT_PROGBITS' or not (section['sh_flags'] & 2):
        raise ValueError('trial flag proof section is not allocated data')
    offset = symbol['st_value'] - section['sh_addr']
    data = section.data()
    if offset < 0 or offset + symbol['st_size'] > len(data):
        raise ValueError('trial flag proof falls outside its section')
    if data[offset:offset + symbol['st_size']] != EXPECTED:
        raise ValueError('trial flag proof bytes differ')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('elf')
    args = p.parse_args()
    with open(args.elf, 'rb') as stream:
        verify(ELFFile(stream))
    print('linked early trial flag proof exact symbol bytes PASS')


if __name__ == '__main__':
    main()
