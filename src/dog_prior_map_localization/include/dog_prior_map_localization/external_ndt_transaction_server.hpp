#pragma once

#include <memory>

namespace dog_prior_map_localization {

class ExternalNdtTransactionServer {
 public:
  ExternalNdtTransactionServer();
  ~ExternalNdtTransactionServer();
  ExternalNdtTransactionServer(const ExternalNdtTransactionServer&) = delete;
  ExternalNdtTransactionServer& operator=(const ExternalNdtTransactionServer&) = delete;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_prior_map_localization
