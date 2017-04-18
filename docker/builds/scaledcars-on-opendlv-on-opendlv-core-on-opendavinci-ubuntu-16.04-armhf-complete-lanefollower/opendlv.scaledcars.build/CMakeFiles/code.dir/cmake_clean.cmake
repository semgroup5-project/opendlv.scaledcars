file(REMOVE_RECURSE
  "CMakeFiles/code"
  "CMakeFiles/code-complete"
  "code-prefix/src/code-stamp/code-install"
  "code-prefix/src/code-stamp/code-mkdir"
  "code-prefix/src/code-stamp/code-download"
  "code-prefix/src/code-stamp/code-update"
  "code-prefix/src/code-stamp/code-patch"
  "code-prefix/src/code-stamp/code-configure"
  "code-prefix/src/code-stamp/code-build"
  "code-prefix/src/code-stamp/code-test"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/code.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
