#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace mesh {

/**
 * \brief  Per-node block list for FLOOD packets, keyed on the relay path.
 *
 *  A flooded packet's `path[]` is appended to by every repeater that forwards
 *  it (its own 1..3 byte public-key prefix), so the LAST entry is the node that
 *  handed us this copy, and entry 0 is the first repeater that relayed it.
 *  Blocking "everything that came through repeater X" is therefore a match on
 *  the last path hash. (See Mesh::getPathHashSize() for the entry width.)
 *
 *  What this cannot do, by design of the protocol:
 *   - DIRECT packets carry the *remaining* route (path[0] == us); the node that
 *     transmitted to us is not recorded, so they cannot be filtered this way.
 *   - a packet ORIGINATED by X starts with an empty path, so X never appears in
 *     it; only traffic X *relayed* can be matched.
 *   - path entries are unauthenticated; this is traffic shaping, not security.
 *
 *  Entries store only the leading bytes of a public key (the same bytes that
 *  appear in `path[]`), so 16 nodes cost 64 bytes of RAM.
 */
class PathFilter {
public:
  static const uint8_t MAX_NODES = 16;
  static const uint8_t MAX_HASH = 3;    // path hashes are 1..3 bytes

  enum Mode : uint8_t {
    MODE_OFF = 0,     // no filtering
    MODE_LAST = 1,    // drop if the previous hop (last path hash) is listed
    MODE_ANY = 2,     // drop if any hop in the path is listed
  };

  struct Entry {
    uint8_t prefix[MAX_HASH];
    uint8_t len;      // meaningful bytes, 1..MAX_HASH
  };

  PathFilter() { reset(); }

  void reset() {
    _count = 0;
    _mode = MODE_OFF;
    _min_hash = 1;
  }

  bool isEnabled() const { return _mode != MODE_OFF && _count > 0; }
  uint8_t getMode() const { return _mode; }
  uint8_t getMinHash() const { return _min_hash; }
  uint8_t getCount() const { return _count; }
  const Entry* getEntry(uint8_t idx) const { return idx < _count ? &_nodes[idx] : NULL; }

  bool setMode(uint8_t mode) {
    if (mode > MODE_ANY) return false;
    _mode = mode;
    return true;
  }

  /** Refuse to enforce on paths whose hashes are shorter than this many bytes
   *  (a 1-byte match is a 1/256 false positive for every other repeater). */
  void setMinHash(uint8_t n) { _min_hash = (n < 1) ? 1 : (n > MAX_HASH ? MAX_HASH : n); }

  void clear() { _count = 0; }

  bool contains(const uint8_t* prefix, uint8_t len) const { return indexOf(prefix, len) >= 0; }

  /** Add a node, given any prefix of its public key (only the first 3 bytes
   *  are kept). Re-adding with a longer prefix upgrades the stored entry. */
  int add(const uint8_t* pub_prefix, uint8_t len) {
    if (len < 1) return _count;
    if (len > MAX_HASH) len = MAX_HASH;

    int existing = indexOf(pub_prefix, len);
    if (existing >= 0) {
      Entry* e = &_nodes[existing];
      if (len > e->len) {
        memset(e->prefix, 0, sizeof(e->prefix));
        memcpy(e->prefix, pub_prefix, len);
        e->len = len;
      }
      return _count;
    }
    if (_count < MAX_NODES) {
      Entry* e = &_nodes[_count++];
      memset(e->prefix, 0, sizeof(e->prefix));
      memcpy(e->prefix, pub_prefix, len);
      e->len = len;
    }
    return _count;
  }

  bool remove(const uint8_t* prefix, uint8_t len) {
    int i = indexOf(prefix, len);
    if (i < 0) return false;
    for (uint8_t k = i; k + 1 < _count; k++) _nodes[k] = _nodes[k + 1];
    _count--;
    return true;
  }

  /** Match one path hash. An entry only matches if it knows at least as many
   *  bytes as the hash on the wire (a 1-byte entry never matches a 3-byte hash
   *  on its first byte alone). */
  bool matchesHash(const uint8_t* hash, uint8_t hash_size) const {
    if (hash_size < 1 || hash_size > MAX_HASH) return false;
    for (uint8_t i = 0; i < _count; i++) {
      const Entry* e = &_nodes[i];
      if (e->len >= hash_size && memcmp(e->prefix, hash, hash_size) == 0) return true;
    }
    return false;
  }

  /**
   * \brief  Should this inbound FLOOD packet be dropped (not re-transmitted)?
   * \param  path        Packet::path
   * \param  hash_count  Packet::getPathHashCount()
   * \param  hash_size   Packet::getPathHashSize()   (1..3)
   *
   * Callers must drop BEFORE the packet is marked as 'seen', so a copy that
   * arrives over an allowed path can still be used.
   */
  bool isBlocked(const uint8_t* path, uint8_t hash_count, uint8_t hash_size) const {
    if (!isEnabled()) return false;
    if (hash_count == 0 || hash_size < _min_hash) return false;   // cannot attribute

    if (_mode == MODE_LAST) {
      return matchesHash(path + (uint16_t)(hash_count - 1) * hash_size, hash_size);
    }
    for (uint8_t i = 0; i < hash_count; i++) {
      if (matchesHash(path + (uint16_t)i * hash_size, hash_size)) return true;
    }
    return false;
  }

  size_t getSerializedSize() const { return 8 + (size_t)_count * sizeof(Entry); }

  size_t serialize(uint8_t* dest, size_t max_len) const {
    size_t need = getSerializedSize();
    if (max_len < need) return 0;

    uint8_t* dp = dest;
    memcpy(dp, "MCPF", 4); dp += 4;          // magic
    *dp++ = 1;                                // version
    *dp++ = _mode;
    *dp++ = _min_hash;
    *dp++ = _count;
    memcpy(dp, _nodes, (size_t)_count * sizeof(Entry));
    return need;
  }

  bool deserialize(const uint8_t* src, size_t len) {
    if (len < 8 || memcmp(src, "MCPF", 4) != 0) return false;
    if (src[4] != 1) return false;

    uint8_t mode = src[5];
    uint8_t min_hash = src[6];
    uint8_t count = src[7];
    if (mode > MODE_ANY || count > MAX_NODES) return false;
    if (len < 8 + (size_t)count * sizeof(Entry)) return false;

    for (uint8_t i = 0; i < count; i++) {     // validate before mutating
      const Entry* e = (const Entry*)(src + 8) + i;
      if (e->len < 1 || e->len > MAX_HASH) return false;
    }

    reset();
    _mode = mode;
    setMinHash(min_hash);
    _count = count;
    memcpy(_nodes, src + 8, (size_t)count * sizeof(Entry));
    return true;
  }

private:
  Entry _nodes[MAX_NODES];
  uint8_t _count;
  uint8_t _mode;
  uint8_t _min_hash;

  int indexOf(const uint8_t* prefix, uint8_t len) const {
    if (len < 1) return -1;
    if (len > MAX_HASH) len = MAX_HASH;
    for (uint8_t i = 0; i < _count; i++) {
      uint8_t n = _nodes[i].len < len ? _nodes[i].len : len;
      if (memcmp(_nodes[i].prefix, prefix, n) == 0) return i;
    }
    return -1;
  }
};

}  // namespace mesh
