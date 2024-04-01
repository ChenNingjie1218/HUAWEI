#include "param.h"

DynamicParam* DynamicParam::instance_ = nullptr;

DynamicParam*& DynamicParam::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new DynamicParam();
  }
  return instance_;
}