// Input-only instrumentation shared by the original and research replay builds.
#pragma once
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <openssl/evp.h>

template<class Cloud>
std::string xyz_sha256(const Cloud& points) {
  EVP_MD_CTX* digest=EVP_MD_CTX_new();
  if (!digest || EVP_DigestInit_ex(digest,EVP_sha256(),nullptr)!=1)
    throw std::runtime_error("digest initialization");
  for (const auto& point : points) {
    const float xyz[3]={point.x,point.y,point.z};
    if (EVP_DigestUpdate(digest,xyz,sizeof(xyz))!=1) throw std::runtime_error("digest update");
  }
  unsigned char data[EVP_MAX_MD_SIZE];
  unsigned int length=0;
  if (EVP_DigestFinal_ex(digest,data,&length)!=1) throw std::runtime_error("digest final");
  EVP_MD_CTX_free(digest);
  std::ostringstream hex;
  hex << std::setfill('0') << std::hex;
  for (unsigned i=0;i<length;++i) hex << std::setw(2) << static_cast<unsigned>(data[i]);
  return hex.str();
}

template<class Cloud>
void record_cloud(const char* label, unsigned long long stamp, const Cloud& points) {
  const char* path=std::getenv("P64_R1_INPUT_RECEIPT");
  if (!path) return;
  std::ofstream out(path,std::ios::app);
  out << label << ',' << stamp << ',' << points.size() << ','
      << points.is_dense << ',' << xyz_sha256(points) << '\n';
  if (!out) throw std::runtime_error("input receipt write failed");
}
