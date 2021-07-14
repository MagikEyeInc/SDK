/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"

SCENARIO("core_offline") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  THEN("device ApiType values") {
    CHECK(mke::device::ApiType::isValid("MkE") == true);
    CHECK(mke::device::ApiType::isValid("MagikEyeOne") == false); // obsolete device type (not ApiType)
    CHECK(mke::device::ApiType::isValid("MagikEyeTwo") == false); // obsolete device type (not ApiType)
    CHECK(mke::device::ApiType::isValid("wrongValue") == false);
    CHECK(mke::device::ApiType::isValid("") == false);
  }

  THEN("device ApiType enum conversions") {
    int no_types;
    auto type_strs = mke::device::ApiType::typeStrings(no_types);
    for (int i = 0; i < no_types; i++) {
      const char* const type_str = type_strs[i];
      std::string type = std::string(type_str);
      CHECK(type != "");
      CHECK(mke::device::ApiType::isValid(type_str) == true);
      auto type_enum = mke::device::ApiType::toEnum(type_str);
      auto type_str2 = mke::device::ApiType::toString(type_enum);
      CHECK(type == std::string(type_str2));
    }
  }
}
