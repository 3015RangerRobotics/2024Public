enum VarType {
  number,
  string,
  boolean,
}

VarType varTypeFromString(String type) {
  return VarType.values.firstWhere((e) => e.name == type);
}
