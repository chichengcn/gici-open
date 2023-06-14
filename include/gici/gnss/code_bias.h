/**
* @Function: Code bias handler
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <mutex>

#include "gici/utility/rtklib_safe.h"

namespace gici {

// TGD and ISC types
enum class TgdIscType {
  None,
  GpsTgd,
  GlonassTgd,
  GalileoBgdE1E5a,
  GalileoBgdE1E5b,
  BdsTgdB1B3,
  BdsTgdB2B3
};

// Code bias handle
class CodeBias {
public:
  // Map from system to base frequency pair. If base on IF combination, put the code channel
  // with higher frequency on LHS and another on RHS. If base on single frequency, put it on
  // LHS and set the RHS as CODE_NONE.
  using BaseFrequencies = std::map<char, std::pair<int, int>>;
  // Map from PRN and code type to bias
  using BiasMap = std::unordered_map<std::string, std::unordered_map<int, double>>;

  CodeBias() { setDefaultBase(); }
  CodeBias(const BaseFrequencies& bases) : bases_(bases)
  { setDefaultBase(); setDefaultDcbs(); }
  ~CodeBias() { }

  // DCB storage (DCB = code2 - code1)
  struct Dcb {
    int code1, code2;
    double value;
    double std;
  };

  // TGD and ISC storage
  struct TgdIsc {
    TgdIscType type;
    double value;
  };

  // Zero-differenced code bias storage
  struct Zdcb {
    int code;
    double value;
  };

  // Set default base frequencies if not setted
  void setDefaultBase();

  // Set default DCBs
  void setDefaultDcbs();

  // Set Differential Code Bias (DCB)
  void setDcb(const std::string prn, 
    const int code, const int code_base, 
    const double value);

  // Set Time Group Delay (TGD) or Inter-System Corrections (ISC)
  void setTgdIsc(const std::string prn, 
    const TgdIscType type, const double value);

  // Set zero-difference code bias
  void setZdcb(const std::string prn, 
    const int code, const double value);

  // Arrange DCBs, TGDs, and ISCs to base frequencies
  void arrangeToBases();

  // Get code bias correction in relative with base frequency
  // Apply the code bias by: P_corrected = P_raw + bias
  // For the broadcast ephemeris, the base frequencies for each system are:
  // GPS: L1P/L2P IF combination (L1W/L2W)
  // GLONASS: L1P/L2P IF combination
  // Galileo: E1/E5a IF combination (INAV)
  // BDS: B3I
  // For the precise ephemeris, user should specify the base frequencies, in 
  // defualt, we use the following configures:
  // GPS: L1P/L2P IF combination (L1W/L2W)
  // GLONASS: L1P/L2P IF combination
  // Galileo: E1/E5a IF combination
  // BDS: B1I/B3I IF combination
  double getCodeBias(const std::string prn, 
    const int code, const bool accept_coarse = true);

  // Get base frequencies
  const BaseFrequencies& getBase() const { return bases_; }

private: 
  // Arrange DCBs, ZDCBs and TGDs to base frequencies
  void arrange();

  // Arrange all source DCBs to biases
  void arrangeAllSourceDcbs(
    std::multimap<std::string, Dcb>& all_source_dcbs, BiasMap& biases, 
    bool clear_update_flag);

  // Put DCBs to all source DCBs
  void putDcbsToAllSourceDcbs();

  // Put ZDCBs to all source DCBs
  void putZdcbsToAllSourceDcbs();

  // Put default DCBs to all source DCBs
  void putDefaultDcbsToAllSourceDcbs();

  // Put TGDs to all source DCBs
  void putTgdsToAllSourceDcbs();

  // Check if a code exists in biases_
  bool checkExist(const std::string prn, const int code, BiasMap& biases);

private:
  // we always prefer to use DCB other than TGD if redundant
  std::multimap<std::string, Dcb> dcbs_;
  std::multimap<std::string, TgdIsc> tgds_;
  std::multimap<std::string, Zdcb> zdcbs_;
  std::multimap<std::string, Dcb> default_dcbs_;
  std::multimap<std::string, Dcb> all_source_dcbs_;
  std::multimap<std::string, Dcb> all_source_dcbs_coarse_;
  std::unordered_map<std::string, bool> biases_updated_;

  // Base frequency 
  BaseFrequencies bases_;

  // Code biases in relative with base frequencies
  BiasMap biases_;
  BiasMap biases_coarse_;
  bool biases_initialized_ = false;

  // Locker
  std::mutex mutex_;
};

using CodeBiasPtr = std::shared_ptr<CodeBias>;

} // namespace gici