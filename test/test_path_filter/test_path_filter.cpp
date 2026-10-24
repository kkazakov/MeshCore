#include <gtest/gtest.h>
#include <helpers/PathFilter.h>

using mesh::PathFilter;

TEST(PathFilter, DisabledByDefault) {
  PathFilter f;
  uint8_t path[] = { 0x11, 0x22, 0x33 };
  f.add(path, 1);
  EXPECT_EQ(f.getMode(), PathFilter::MODE_OFF);
  EXPECT_FALSE(f.isEnabled());
  EXPECT_FALSE(f.isBlocked(path, 3, 1));
}

TEST(PathFilter, LastHopMatchesPreviousRelayOnly) {
  PathFilter f;
  uint8_t blocked[] = { 0xBB };

  f.add(blocked, 1);
  f.setMode(PathFilter::MODE_LAST);

  // 0xBB handed us this copy
  uint8_t via_bb[] = { 0xAA, 0xCC, 0xBB };
  EXPECT_TRUE(f.isBlocked(via_bb, 3, 1));

  // 0xBB relayed it earlier, but 0xDD handed it to us
  uint8_t via_dd[] = { 0xAA, 0xBB, 0xDD };
  EXPECT_FALSE(f.isBlocked(via_dd, 3, 1));
}

TEST(PathFilter, AnyHopMatchesTransit) {
  PathFilter f;
  uint8_t blocked[] = { 0xBB };
  f.add(blocked, 1);
  f.setMode(PathFilter::MODE_ANY);

  uint8_t via_bb[] = { 0xAA, 0xBB, 0xCC };
  EXPECT_TRUE(f.isBlocked(via_bb, 3, 1));

  uint8_t clean[] = { 0xAA, 0xCC };
  EXPECT_FALSE(f.isBlocked(clean, 2, 1));
}

TEST(PathFilter, ZeroHopIsNeverAttributed) {
  PathFilter f;
  uint8_t blocked[] = { 0xBB };
  f.add(blocked, 1);
  f.setMode(PathFilter::MODE_LAST);

  uint8_t empty[] = { 0 };
  EXPECT_FALSE(f.isBlocked(empty, 0, 1));
}

TEST(PathFilter, MinHashRefusesShortPaths) {
  PathFilter f;
  uint8_t blocked[] = { 0xBB };
  f.add(blocked, 1);
  f.setMode(PathFilter::MODE_LAST);
  f.setMinHash(2);

  uint8_t one_byte[] = { 0xAA, 0xBB };
  EXPECT_FALSE(f.isBlocked(one_byte, 2, 1));   // 1-byte hashes: not enforced

  uint8_t two_byte[] = { 0xAA, 0xBB, 0xBB, 0xCC };
  uint8_t blocked2[] = { 0xBB, 0xCC };
  f.add(blocked2, 2);
  EXPECT_TRUE(f.isBlocked(two_byte, 2, 2));
}

TEST(PathFilter, EntryNeverMatchesLongerHash) {
  PathFilter f;
  uint8_t one_byte[] = { 0xBB };
  uint8_t hash3[] = { 0xBB, 0x01, 0x02 };

  f.add(one_byte, 1);
  EXPECT_FALSE(f.matchesHash(hash3, 3));   // would be a 1/256 match
  EXPECT_TRUE(f.matchesHash(hash3, 1));
}

TEST(PathFilter, AddUpgradesPrefixPrecision) {
  PathFilter f;
  uint8_t short_pref[] = { 0xAA };
  uint8_t long_pref[] = { 0xAA, 0xBB, 0xCC };

  f.add(short_pref, 1);
  f.add(long_pref, 3);
  EXPECT_EQ(f.getCount(), 1);
  EXPECT_EQ(f.getEntry(0)->len, 3);
  EXPECT_TRUE(f.contains(long_pref, 3));
}

TEST(PathFilter, RemoveAndClear) {
  PathFilter f;
  uint8_t a[] = { 0xAA, 0x01 };
  uint8_t b[] = { 0xBB, 0x02 };

  f.add(a, 2);
  f.add(b, 2);
  EXPECT_TRUE(f.remove(a, 2));
  EXPECT_EQ(f.getCount(), 1);
  EXPECT_FALSE(f.contains(a, 2));
  EXPECT_TRUE(f.contains(b, 2));

  f.clear();
  EXPECT_EQ(f.getCount(), 0);
  EXPECT_FALSE(f.isEnabled());
}

TEST(PathFilter, SerializeRoundTrip) {
  PathFilter f;
  uint8_t a[] = { 0xAA, 0xBB };
  uint8_t b[] = { 0x01, 0x02, 0x03 };
  f.add(a, 2);
  f.add(b, 3);
  f.setMode(PathFilter::MODE_ANY);
  f.setMinHash(2);

  uint8_t buf[128];
  size_t n = f.serialize(buf, sizeof(buf));
  ASSERT_GT(n, 0u);

  PathFilter g;
  ASSERT_TRUE(g.deserialize(buf, n));
  EXPECT_EQ(g.getMode(), PathFilter::MODE_ANY);
  EXPECT_EQ(g.getMinHash(), 2);
  EXPECT_EQ(g.getCount(), 2);

  uint8_t path[] = { 0x11, 0x22, 0xAA, 0xBB };
  EXPECT_TRUE(g.isBlocked(path, 2, 2));   // second hop matches the 0xAA,0xBB entry
}

TEST(PathFilter, DeserializeRejectsCorrupt) {
  PathFilter f;
  uint8_t buf[16];
  memset(buf, 0, sizeof(buf));

  EXPECT_FALSE(f.deserialize(buf, sizeof(buf)));   // bad magic

  memcpy(buf, "MCPF", 4);
  buf[4] = 1;   // version
  buf[5] = 0;   // mode
  buf[6] = 1;   // min hash
  buf[7] = 1;   // count
  buf[8] = 0xAA; buf[9] = 0; buf[10] = 0; buf[11] = 0;   // len 0
  EXPECT_FALSE(f.deserialize(buf, 12));

  buf[11] = 4;  // len > MAX_HASH
  EXPECT_FALSE(f.deserialize(buf, 12));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
