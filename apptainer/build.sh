apptainer build --fakeroot \
  --build-arg UBUNTU_VERSION=24.04 \
  --build-arg NVIDIA_CUDA_VERSION=12.9.1 \
  --build-arg COLMAP_GIT_REPOSITORY=https://github.com/SiyuChen1/colmap.git \
  --build-arg COLMAP_GIT_COMMIT=new_feature/rig_verification_timer \
  --build-arg CUDA_ARCHITECTURES=all-major \
  colmap.sif colmap.def
