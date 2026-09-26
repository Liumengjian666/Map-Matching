#pragma once

#include <openssl/evp.h>

#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace p5_i1 {
constexpr char kExpectedBagSha256[] =
    "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db";
constexpr char kExpectedMapSha256[] =
    "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570";

inline std::string sha256File(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open input for SHA256: " + path);
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (!context) throw std::runtime_error("cannot allocate SHA256 context");
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("cannot initialize SHA256 context");
  }
  std::array<char, 1 << 20> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0 && EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(count)) != 1) {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("SHA256 update failed for: " + path);
    }
  }
  if (!input.eof()) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("read failed while hashing: " + path);
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  const int result = EVP_DigestFinal_ex(context, digest.data(), &length);
  EVP_MD_CTX_free(context);
  if (result != 1 || length != 32) throw std::runtime_error("SHA256 finalization failed: " + path);
  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < length; ++i) hex << std::setw(2) << static_cast<unsigned int>(digest[i]);
  return hex.str();
}

inline void requireFrozenMapSha256(const std::string& path) {
  const std::string actual = sha256File(path);
  if (actual != kExpectedMapSha256)
    throw std::runtime_error("prior-map SHA256 mismatch: actual=" + actual +
                             " expected=" + kExpectedMapSha256);
}

inline void requireFrozenBagSha256(const std::string& path) {
  const std::string actual = sha256File(path);
  if (actual != kExpectedBagSha256)
    throw std::runtime_error("runtime-topic bag SHA256 mismatch: actual=" + actual +
                             " expected=" + kExpectedBagSha256);
}
}  // namespace p5_i1
