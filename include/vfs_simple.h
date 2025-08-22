#pragma once
#include <Arduino.h>
#include "w25q.h"

// Extremely simple append-only "VFS":
// - Region stores CCSDS frames as [len_be(2)][packet bytes]
// - On boot, we find end by scanning from base until we hit erased bytes (0xFFFF)
//   or an invalid length.
// - No journal; clearing = erase whole region.
// - When out of space, we erase region and start from base again.

struct Region {
  const char* name;
  uint32_t base;   // first byte usable
  uint32_t limit;  // end (exclusive)
  uint32_t wptr;   // next write position (absolute)
};

extern Region GPS_FILE;   // 0x00000..0x10000
extern Region ACC_FILE;   // 0x10000..0x20000

// Initialize regions and compute write pointers by scan.
void vfs_init(bool force_clear=false);

// Append a single CCSDS packet framed as [len_be][bytes].
bool vfs_append_packet(Region& R, const uint8_t* pkt, uint16_t len);

// Dump all packets in region as raw bytes to Stream.
void vfs_dump_raw(Stream& out, const Region& R);

// Clear region (erase all sectors) and reset wptr.
void vfs_clear(Region& R);

// Helpers to iterate packets (for CSV dump)
bool vfs_first(const Region& R, uint32_t& rd);
bool vfs_next (const Region& R, uint32_t& rd, uint16_t& L);