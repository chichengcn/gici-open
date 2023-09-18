/**
* @Function: Code bias handler
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/code_bias.h"

#include <vector>
#include <algorithm>
#include <Eigen/Core>

#include "gici/gnss/gnss_common.h"

namespace gici {

// Set default base frequencies if not setted
void CodeBias::setDefaultBase()
{
  mutex_.lock();
  if (bases_.find('G') == bases_.end()) {
    bases_.insert(std::make_pair('G', std::make_pair(CODE_L1W, CODE_L2W)));
  }
  if (bases_.find('R') == bases_.end()) {
    bases_.insert(std::make_pair('R', std::make_pair(CODE_L1P, CODE_L2P)));
  }
  if (bases_.find('E') == bases_.end()) {
    bases_.insert(std::make_pair('E', std::make_pair(CODE_L1C, CODE_L5Q)));
  }
  if (bases_.find('C') == bases_.end()) {
    bases_.insert(std::make_pair('C', std::make_pair(CODE_L6I, CODE_NONE)));
  }
  mutex_.unlock();
}

// Set default DCBs
void CodeBias::setDefaultDcbs()
{
  mutex_.lock();
  Dcb default_dcb;
  default_dcb.value = 0.0;
  default_dcb.std = 1.0;
  // GPS L1
  default_dcb.code1 = CODE_L1W;
  default_dcb.code2 = CODE_L1C;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L1S;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L1L;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L1X;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L1P;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L1Y;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L1M;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  // GPS L2
  default_dcb.code1 = CODE_L2W;
  default_dcb.code2 = CODE_L2C;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2D;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2S;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2L;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2X;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2P;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2Y;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L2M;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  // GPS L5
  default_dcb.code1 = CODE_L5Q;
  default_dcb.code2 = CODE_L5I;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  default_dcb.code2 = CODE_L5X;
  default_dcbs_.insert(std::make_pair("Gxx", default_dcb));
  // GLONASS G1
  default_dcb.code1 = CODE_L1P;
  default_dcb.code2 = CODE_L1C;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  // GLONASS G2
  default_dcb.code1 = CODE_L2P;
  default_dcb.code2 = CODE_L2C;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  // GLONASS G1A
  default_dcb.code1 = CODE_L4A;
  default_dcb.code2 = CODE_L4B;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  default_dcb.code2 = CODE_L4X;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  // GLONASS G2A
  default_dcb.code1 = CODE_L6A;
  default_dcb.code2 = CODE_L6B;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  default_dcb.code2 = CODE_L6X;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  // GLONASS G3
  default_dcb.code1 = CODE_L3I;
  default_dcb.code2 = CODE_L3Q;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  default_dcb.code2 = CODE_L3X;
  default_dcbs_.insert(std::make_pair("Rxx", default_dcb));
  // GAL E1
  default_dcb.code1 = CODE_L1C;
  default_dcb.code2 = CODE_L1A;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L1B;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L1X;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L1Z;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  // GAL E5A
  default_dcb.code1 = CODE_L5Q;
  default_dcb.code2 = CODE_L5I;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L5X;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  // GAL E5B
  default_dcb.code1 = CODE_L7Q;
  default_dcb.code2 = CODE_L7I;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L7X;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  // GAL E5
  default_dcb.code1 = CODE_L8Q;
  default_dcb.code2 = CODE_L8I;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L8X;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  // GAL E6
  default_dcb.code1 = CODE_L6C;
  default_dcb.code2 = CODE_L6A;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L6B;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L6X;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  default_dcb.code2 = CODE_L6Z;
  default_dcbs_.insert(std::make_pair("Exx", default_dcb));
  // BDS B1
  default_dcb.code1 = CODE_L2I;
  default_dcb.code2 = CODE_L2Q;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L2X;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B1C
  default_dcb.code1 = CODE_L1P;
  default_dcb.code2 = CODE_L1D;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L1X;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B1A
  default_dcb.code1 = CODE_L1L;
  default_dcb.code2 = CODE_L1S;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L1Z;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B2A
  default_dcb.code1 = CODE_L5P;
  default_dcb.code2 = CODE_L5D;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L5X;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B2
  default_dcb.code1 = CODE_L7I;
  default_dcb.code2 = CODE_L7Q;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L7X;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B2B
  default_dcb.code1 = CODE_L7P;
  default_dcb.code2 = CODE_L7D;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L7Z;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B2AB
  default_dcb.code1 = CODE_L8P;
  default_dcb.code2 = CODE_L8D;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L8X;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  // BDS B3
  default_dcb.code1 = CODE_L6I;
  default_dcb.code2 = CODE_L6Q;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  default_dcb.code2 = CODE_L6X;
  default_dcbs_.insert(std::make_pair("Cxx", default_dcb));
  mutex_.unlock();
}

// Set Differential Code Bias (DCB)
void CodeBias::setDcb(const std::string prn, 
    const int code, const int code_base, 
    const double value)
{
  CHECK(code != code_base);
  CHECK(prn.size() == 3);

  // check if exist
  bool found = false;
  auto it_dcb = dcbs_.lower_bound(prn);
  for ( ;it_dcb != dcbs_.upper_bound(prn); it_dcb++) {
    if (it_dcb->second.code1 == code_base && 
        it_dcb->second.code2 == code) {
      found = true;
      break;
    }
  }
  
  // Add to handle
  if (found) {
    if (!checkEqual(it_dcb->second.value, value, 0.01)) {
      it_dcb->second.value = value;
      biases_updated_[prn] = true;
    }
  }
  else {
    Dcb dcb;
    dcb.code1 = code_base;
    dcb.code2 = code;
    dcb.value = value;
    dcb.std = 0.01;
    dcbs_.insert(std::make_pair(prn, dcb));
    biases_updated_[prn] = true;
  }
}

// Set Time Group Delay (TGD) or Inter-System Corrections (ISC)
void CodeBias::setTgdIsc(const std::string prn, 
  const TgdIscType type, const double value)
{
  // check if exist
  bool found = false;
  auto it_tgd = tgds_.lower_bound(prn);
  for ( ;it_tgd != tgds_.upper_bound(prn); it_tgd++) {
    if (it_tgd->second.type == type) {
      found = true;
      break;
    }
  }
  
  // Add to handle
  if (found) {
    if (!checkEqual(it_tgd->second.value, value * CLIGHT, 0.01)) {
      it_tgd->second.value = value * CLIGHT;
      biases_updated_[prn] = true;
    }
  } 
  else {
    TgdIsc tgd;
    tgd.type = type;
    tgd.value = value * CLIGHT;
    tgds_.insert(std::make_pair(prn, tgd));
    biases_updated_[prn] = true;
  }
}

// Set zero-difference code bias
void CodeBias::setZdcb(const std::string prn, 
  const int code, const double value)
{
  CHECK(prn.size() == 3);

  // check if exist
  bool found = false;
  auto it_zdcb = zdcbs_.lower_bound(prn);
  for ( ;it_zdcb != zdcbs_.upper_bound(prn); it_zdcb++) {
    if (it_zdcb->second.code == code) {
      found = true;
      break;
    }
  }
  
  // Add to handle
  bool updated = false;
  if (found) {
    if (!checkEqual(it_zdcb->second.value, value, 0.01)) {
      it_zdcb->second.value = value;
      biases_updated_[prn] = true;
    }
  }
  else {
    Zdcb zdcb;
    zdcb.code = code;
    zdcb.value = value;
    zdcbs_.insert(std::make_pair(prn, zdcb));
    biases_updated_[prn] = true;
  }
}

// Arrange DCBs, TGDs, and ISCs to base frequencies
void CodeBias::arrangeToBases()
{
  mutex_.lock();

  // Arrange to code biases
  arrange();

  mutex_.unlock();

  biases_initialized_ = true;
}

// Get code bias correction
double CodeBias::getCodeBias(const std::string prn, 
  const int code, const bool accept_coarse)
{
  if (!biases_initialized_) return 0.0;

  // Get code bias
  mutex_.lock();
  if (!accept_coarse) 
  {
    if (biases_.find(prn) != biases_.end()) {
      std::unordered_map<int, double> biases_sat = biases_.at(prn);
      if (biases_sat.find(code) != biases_sat.end()) {
        double bias = biases_sat.at(code);
        // we need check if it is zero (invalid) outside
        if (bias == 0.0) bias = 1e-4;
        mutex_.unlock();
        return bias;
      }
    }
  }
  else
  {
    if (biases_coarse_.find(prn) != biases_coarse_.end()) {
      std::unordered_map<int, double> biases_sat = biases_coarse_.at(prn);
      if (biases_sat.find(code) != biases_sat.end()) {
        double bias = biases_sat.at(code);
        // we need check if it is zero (invalid) outside
        if (bias == 0.0) bias = 1e-4;
        mutex_.unlock();
        return bias;
      }
    }
  }
  mutex_.unlock();

  return 0.0;
}

// Arrange DCBs, ZDCBs and TGDs to base frequencies
void CodeBias::arrange()
{
  // Clear all source updated DCBs storage
  for (auto it = all_source_dcbs_.begin(); it != all_source_dcbs_.end(); ) {
    if (biases_updated_.at(it->first)) {
      it = all_source_dcbs_.erase(it);
    }
    else it++;
  }
  for (auto it = all_source_dcbs_coarse_.begin(); 
            it != all_source_dcbs_coarse_.end(); ) {
    if (biases_updated_.at(it->first)) {
      it = all_source_dcbs_coarse_.erase(it);
    }
    else it++;
  }

  // Put all source biases to DCBs
  putDcbsToAllSourceDcbs();
  putZdcbsToAllSourceDcbs();
  putTgdsToAllSourceDcbs();
  putDefaultDcbsToAllSourceDcbs();

  // Arrange precise biases
  arrangeAllSourceDcbs(all_source_dcbs_, biases_, false);

  // Arrange coarse biases
  arrangeAllSourceDcbs(all_source_dcbs_coarse_, biases_coarse_, true);
}

// Arrange all source DCBs to biases
void CodeBias::arrangeAllSourceDcbs(
  std::multimap<std::string, Dcb>& all_source_dcbs, BiasMap& biases,
  bool clear_update_flag)
{
  // get all PRNs
  std::vector<std::string> prns;
  for (auto dcb : all_source_dcbs) {
    if (!biases_updated_.at(dcb.first)) continue;

    if (prns.size() == 0 || prns.back() != dcb.first) {
      prns.push_back(dcb.first);
    }

    if (clear_update_flag) biases_updated_.at(dcb.first) = false;
  }

  // fill DCBs of every satellites
  for (auto prn : prns) {
    std::vector<int> codes;
    std::vector<Dcb> dcbs;
    for (auto dcb = all_source_dcbs.lower_bound(prn); 
        dcb != all_source_dcbs.upper_bound(prn); dcb++) {
      if (std::find(codes.begin(), codes.end(), dcb->second.code1) 
          == codes.end()) { 
        codes.push_back(dcb->second.code1);
      } 
      if (std::find(codes.begin(), codes.end(), dcb->second.code2) 
          == codes.end()) { 
        codes.push_back(dcb->second.code2);
      } 
      dcbs.push_back(dcb->second);
    }
    if (codes.size() == 0) continue;

    // apply a least-square to convert DCB to code biases
    Eigen::VectorXd x = Eigen::VectorXd::Zero(codes.size());
    Eigen::VectorXd z = Eigen::VectorXd::Zero(dcbs.size() + 1);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(dcbs.size() + 1, dcbs.size() + 1);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dcbs.size() + 1, codes.size());
    for (size_t i = 0; i < dcbs.size(); i++) {
      z(i) = dcbs[i].value;
      W(i, i) = 1.0 / square(dcbs[i].std);
      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == dcbs[i].code1) H(i, j) = -1.0;
        if (codes[j] == dcbs[i].code2) H(i, j) = 1.0;
      }
    }

    // set the base frequency as zero
    std::pair<int, int> base = bases_.at(prn[0]);
    z(dcbs.size()) = 0.0;
    W(dcbs.size()) = 1.0e-2;

    // single frequency base
    if (base.second == CODE_NONE) {
      bool found = false;
      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == base.first) {
          H(dcbs.size(), j) = 1.0; found = true;
        }
      }
      if (!found) {
        LOG(INFO) << "Input DCBs for " << prn << " does not contain base code " 
          << gnss_common::codeTypeToRinexType(prn[0], base.first) << "!";
        continue;
      }
    }
    // ionosphere-free combination base
    else {
      double f1 = gnss_common::codeToFrequency(prn[0], base.first);
      double f2 = gnss_common::codeToFrequency(prn[0], base.second);
      double c1 = square(f1) / (square(f1) - square(f2));
      double c2 = -square(f2) / (square(f1) - square(f2));
      bool found_first = false, found_second = false;;
      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == base.first) {
          H(dcbs.size(), j) = c1; found_first = true;
        }
        else if (codes[j] == base.second) {
          H(dcbs.size(), j) = c2; found_second = true;
        }
      }
      if (!found_first) {
        LOG(INFO) << "Input DCBs for " << prn << " does not contain base code " 
          << gnss_common::codeTypeToRinexType(prn[0], base.first) << "!";
        continue;
      }
      if (!found_second) {
        LOG(INFO) << "Input DCBs for " << prn << " does not contain base code " 
          << gnss_common::codeTypeToRinexType(prn[0], base.second) << "!";
        continue;
      }
    }

    // Check rank
    if (checkZero((H.transpose() * H).determinant())) {
      LOG(INFO) << "Input DCBs are not closed for " << prn 
                 << "! H = " << std::endl << H;
      continue;
    }

    // solve
    x = (H.transpose() * W * H).inverse() * H.transpose() * W * z;
    
    // fill bias handle
    for (size_t i = 0; i < codes.size(); i++) {
      if (biases.find(prn) == biases.end()) {
        biases.insert(
          std::make_pair(prn, std::unordered_map<int, double>()));
      }
      biases.at(prn).insert(std::make_pair(codes[i], x(i)));
    }
  }
}

// Put DCBs to all source DCBs
void CodeBias::putDcbsToAllSourceDcbs()
{
  for (auto dcb : dcbs_) {
    if (!biases_updated_.at(dcb.first)) continue;
    all_source_dcbs_.insert(dcb);
    all_source_dcbs_coarse_.insert(dcb);
  }
}

// Put ZDCBs to all source DCBs
void CodeBias::putZdcbsToAllSourceDcbs()
{
  std::string current_prn;
  std::vector<Zdcb> zdcbs;
  for (auto it = zdcbs_.begin(); it != zdcbs_.end(); it++) {
    std::string prn = it->first;
    Zdcb zdcb = it->second;

    if (!biases_updated_.at(prn)) continue;

    zdcbs.push_back(zdcb);
    if (current_prn.empty()) current_prn = prn;

    std::multimap<std::string, Zdcb>::iterator next = it;
    next++;
    if (next == zdcbs_.end() || next->first != current_prn) {
      Dcb dcb;
      if (zdcbs.size() > 1) 
      for (size_t i = 1; i < zdcbs.size(); i++) {
        dcb.code1 = zdcbs[0].code;
        dcb.code2 = zdcbs[i].code;
        dcb.value = zdcbs[i].value - zdcbs[0].value;
        dcb.std = 0.01 + 1.0e-6;  // add 1e-6 to make sure it differs from raw DCBs
        all_source_dcbs_.insert(std::make_pair(current_prn, dcb));
        all_source_dcbs_coarse_.insert(std::make_pair(current_prn, dcb));
      }

      zdcbs.clear();
      if (next != zdcbs_.end()) current_prn = next->first;
      continue;
    }
  }
}

// Put default DCBs to all source DCBs
void CodeBias::putDefaultDcbsToAllSourceDcbs()
{
  // get all PRNs
  std::vector<std::string> prns;
  for (auto dcb : all_source_dcbs_coarse_) {
    if (!biases_updated_.at(dcb.first)) continue;
    
    if (prns.size() == 0 || prns.back() != dcb.first) {
      prns.push_back(dcb.first);
    }
  }

  // add defaults
  for (auto prn : prns) {
    const char system = prn[0];
    std::vector<int> phases;
    for (auto dcb = all_source_dcbs_coarse_.lower_bound(prn); 
        dcb != all_source_dcbs_coarse_.upper_bound(prn); dcb++) {
      int phase1 = gnss_common::getPhaseID(system, dcb->second.code1);
      int phase2 = gnss_common::getPhaseID(system, dcb->second.code2);
      if (std::find(phases.begin(), phases.end(), phase1) == phases.end()) { 
        phases.push_back(phase1);
      } 
      if (std::find(phases.begin(), phases.end(), phase2) == phases.end()) { 
        phases.push_back(phase2);
      } 
    }
    if (phases.size() == 0) continue;

    for (auto default_dcb : default_dcbs_) {
      const char system = prn[0];
      if (default_dcb.first[0] != system) continue;
      int phase1 = gnss_common::getPhaseID(system, default_dcb.second.code1);
      int phase2 = gnss_common::getPhaseID(system, default_dcb.second.code2);
      if ((std::find(phases.begin(), phases.end(), phase1) == phases.end()) && 
          (std::find(phases.begin(), phases.end(), phase2) == phases.end())) continue;
      all_source_dcbs_coarse_.insert(std::make_pair(prn, default_dcb.second));
    }
  }
}

// Put TGDs to all source DCBs
void CodeBias::putTgdsToAllSourceDcbs()
{
  for (auto it = tgds_.begin(); it != tgds_.end(); it++) {
    std::string prn = it->first;
    TgdIsc tgd = it->second;

    if (!biases_updated_.at(prn)) continue;

    Dcb dcb;
    if (tgd.type == TgdIscType::None) continue;
    else if (tgd.type == TgdIscType::GpsTgd) {
      dcb.code1 = CODE_L1W;
      dcb.code2 = CODE_L2W;
      double f1 = gnss_common::codeToFrequency(prn[0], dcb.code1);
      double f2 = gnss_common::codeToFrequency(prn[0], dcb.code2);
      dcb.value = tgd.value * (1 - square(f1 / f2));
    }
    else if (tgd.type == TgdIscType::GlonassTgd) {
      dcb.code1 = CODE_L1P;
      dcb.code2 = CODE_L2P;
      dcb.value = -tgd.value;
    }
    else if (tgd.type == TgdIscType::GalileoBgdE1E5a) {
      dcb.code1 = CODE_L1C;
      dcb.code2 = CODE_L5Q;
      double f1 = gnss_common::codeToFrequency(prn[0], dcb.code1);
      double f2 = gnss_common::codeToFrequency(prn[0], dcb.code2);
      dcb.value = tgd.value * (1 - square(f1 / f2));
    }
    else if (tgd.type == TgdIscType::GalileoBgdE1E5b) {
      // F/NAV
      dcb.code1 = CODE_L1C;
      dcb.code2 = CODE_L7Q;
      double f1 = gnss_common::codeToFrequency(prn[0], dcb.code1);
      double f2 = gnss_common::codeToFrequency(prn[0], dcb.code2);
      dcb.value = tgd.value * (1 - square(f1 / f2));
    }
    else if (tgd.type == TgdIscType::BdsTgdB1B3) {
      dcb.code1 = CODE_L2I;
      dcb.code2 = CODE_L6I;
      dcb.value = tgd.value;
    }
    else if (tgd.type == TgdIscType::BdsTgdB2B3) {
      dcb.code1 = CODE_L7I;
      dcb.code2 = CODE_L6I;
      dcb.value = tgd.value;
    }
    dcb.std = 0.3;

    all_source_dcbs_coarse_.insert(std::make_pair(prn, dcb));
  }
}

// Check if a code exists in biases_
bool CodeBias::checkExist(const std::string prn, const int code, BiasMap& biases)
{
  if (biases.find(prn) == biases.end()) return false;
  std::unordered_map<int, double>& code_to_bias = biases.at(prn);
  if (code_to_bias.find(code) != code_to_bias.end()) return true;
  return false;
}

}