#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 4)}"

COMMON_CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DCMAKE_CXX_STANDARD=17
)

if [[ -n "${EIGEN3_DIR:-}" ]]; then
  COMMON_CMAKE_ARGS+=("-DEigen3_DIR=${EIGEN3_DIR}")
fi

configure_and_build() {
  local name="$1"
  local src_dir="$2"
  local build_dir="$3"

  echo "Configuring and building ${name} ..."
  cmake -S "${src_dir}" -B "${build_dir}" "${COMMON_CMAKE_ARGS[@]}"

  if ! cmake --build "${build_dir}" -j "${JOBS}"; then
    echo "[ERROR] ${name} 构建失败。"
    if [[ "${name}" == "ext/g2o" ]]; then
      cat <<'MSG'
[ERROR] ext/g2o 构建失败。
检测到的典型阻塞是旧版 g2o 与当前编译器/标准库兼容性问题：
- tr1/unordered_map 头文件不可用
- 旧 C++03 模板语法（如 >>）在当前设置下触发错误
这不是 build.sh 参数问题，需要对 ext/g2o 源码做兼容性补丁。
MSG
    fi
    return 1
  fi
}

cd "${ROOT_DIR}"

configure_and_build "ext/DBoW2" "${ROOT_DIR}/ext/DBoW2" "${ROOT_DIR}/ext/DBoW2/build"
configure_and_build "ext/g2o" "${ROOT_DIR}/ext/g2o" "${ROOT_DIR}/ext/g2o/build"
configure_and_build "ext/Sophus" "${ROOT_DIR}/ext/Sophus" "${ROOT_DIR}/ext/Sophus/build"

if [[ -f "${ROOT_DIR}/Vocabulary/ORBvoc.txt" ]]; then
  echo "Vocabulary/ORBvoc.txt already exists, skip uncompress."
else
  echo "Uncompress vocabulary ..."
  tar -xf "${ROOT_DIR}/Vocabulary/ORBvoc.txt.tar.gz" -C "${ROOT_DIR}/Vocabulary"
fi

configure_and_build "ORB_SLAM3" "${ROOT_DIR}" "${ROOT_DIR}/build"

echo "Build finished."
