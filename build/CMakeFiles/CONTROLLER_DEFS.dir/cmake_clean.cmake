FILE(REMOVE_RECURSE
  "lib/libCONTROLLER_DEFS.pdb"
  "lib/libCONTROLLER_DEFS.a"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/CONTROLLER_DEFS.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
