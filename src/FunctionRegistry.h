#pragma once

#include <Arduino.h>

#include <functional>
#include <type_traits>
#include <vector>

#include "Peripherals/OperationResult.h"

class FunctionRegistry {
 public:
  enum class ExecuteResult { Success, FunctionNotFound, ArgumentError };

 private:
  struct FunctionEntry {
    String name;
    std::function<OperationResult(const std::vector<float>&)> func;
    int argCount;
    bool is_void;

    FunctionEntry(const String& n,
                  std::function<OperationResult(const std::vector<float>&)> f,
                  int ac, bool iv)
        : name(n), func(f), argCount(ac), is_void(iv) {}
  };
  std::vector<FunctionEntry> functions;

  template <typename T>
  struct is_void_return {
    template <typename U>
    static char test(typename std::enable_if<
                     std::is_void<decltype(std::declval<U>()(
                         std::declval<const std::vector<float>&>()))>::value,
                     int>::type);

    template <typename U>
    static long test(...);

    static const bool value = sizeof(test<T>(0)) == 1;
  };

 public:
  template <typename Func>
  void registerFunction(const String& name, Func&& func, int argCount) {
    String upper_name = name;
    upper_name.toUpperCase();

    auto wrapper = [func = std::forward<Func>(func)](
                       const std::vector<float>& args) -> OperationResult {
      if constexpr (is_void_return<decltype(func)>::value) {
        func(args);
        return OperationResult::Success();
      } else {
        return func(args);
      }
    };

    functions.emplace_back(upper_name, wrapper, argCount,
                           is_void_return<Func>::value);
  }

  ExecuteResult execute(const String& name, const std::vector<float>& args,
                        OperationResult& result) const {
    String upper_name = name;
    upper_name.toUpperCase();
    for (const auto& entry : functions) {
      if (entry.name == upper_name) {
        if (entry.argCount > 0 && entry.argCount != args.size()) {
          result = OperationResult::Failure("Argument count mismatch");
          return ExecuteResult::ArgumentError;
        }
        result = entry.func(args);
        return ExecuteResult::Success;
      }
    }
    result = OperationResult::Failure("Function not found");
    return ExecuteResult::FunctionNotFound;
  }
};