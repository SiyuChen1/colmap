// Copyright (c), ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "colmap/estimators/bundle_adjustment.h"
#include "colmap/estimators/covariance.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/scene/camera.h"
#include "colmap/scene/image.h"
#include "colmap/scene/point2d.h"
#include "colmap/scene/point3d.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/scene/reconstruction_io_utils.h"
#include "colmap/scene/track.h"
#include "colmap/util/file.h"
#include "colmap/util/types.h"

#include <fstream>
#include <optional>
#include <ceres/covariance.h>

namespace colmap {

void ReadRigsText(Reconstruction& reconstruction, std::istream& stream) {
  THROW_CHECK(stream.good());

  std::string line;
  std::string item;

  while (std::getline(stream, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    Rig rig;

    // ID
    std::getline(line_stream, item, ' ');
    rig.SetRigId(std::stoull(item));

    // NUM_SENSORS
    std::getline(line_stream, item, ' ');
    const uint64_t num_sensors = std::stoull(item);

    if (num_sensors > 0) {
      // REF_SENSOR
      sensor_t ref_sensor_id;
      std::getline(line_stream, item, ' ');
      ref_sensor_id.type = SensorTypeFromString(item);
      std::getline(line_stream, item, ' ');
      ref_sensor_id.id = std::stoul(item);
      rig.AddRefSensor(ref_sensor_id);
    }

    // SENSORS
    if (num_sensors > 1) {
      for (uint64_t i = 0; i < num_sensors - 1; ++i) {
        sensor_t sensor_id;
        std::getline(line_stream, item, ' ');
        sensor_id.type = SensorTypeFromString(item);
        std::getline(line_stream, item, ' ');
        sensor_id.id = std::stoul(item);

        std::getline(line_stream, item, ' ');
        const bool has_pose = item == "1";

        std::optional<Rigid3d> sensor_from_rig;
        if (has_pose) {
          sensor_from_rig = Rigid3d();

          std::getline(line_stream, item, ' ');
          sensor_from_rig->rotation.w() = std::stold(item);

          std::getline(line_stream, item, ' ');
          sensor_from_rig->rotation.x() = std::stold(item);

          std::getline(line_stream, item, ' ');
          sensor_from_rig->rotation.y() = std::stold(item);

          std::getline(line_stream, item, ' ');
          sensor_from_rig->rotation.z() = std::stold(item);

          std::getline(line_stream, item, ' ');
          sensor_from_rig->translation.x() = std::stold(item);

          std::getline(line_stream, item, ' ');
          sensor_from_rig->translation.y() = std::stold(item);

          std::getline(line_stream, item, ' ');
          sensor_from_rig->translation.z() = std::stold(item);
        }

        rig.AddSensor(sensor_id, sensor_from_rig);
      }
    }

    reconstruction.AddRig(std::move(rig));
  }
}

void ReadRigsText(Reconstruction& reconstruction, const std::string& path) {
  std::ifstream file(path);
  THROW_CHECK_FILE_OPEN(file, path);
  ReadRigsText(reconstruction, file);
}

void ReadCamerasText(Reconstruction& reconstruction, std::istream& stream) {
  THROW_CHECK(stream.good());

  std::string line;
  std::string item;

  while (std::getline(stream, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    struct Camera camera;

    // ID
    std::getline(line_stream, item, ' ');
    camera.camera_id = std::stoul(item);

    // MODEL
    std::getline(line_stream, item, ' ');
    camera.model_id = CameraModelNameToId(item);

    // WIDTH
    std::getline(line_stream, item, ' ');
    camera.width = std::stoll(item);

    // HEIGHT
    std::getline(line_stream, item, ' ');
    camera.height = std::stoll(item);

    // PARAMS
    camera.params.reserve(CameraModelNumParams(camera.model_id));
    while (!line_stream.eof()) {
      std::getline(line_stream, item, ' ');
      camera.params.push_back(std::stold(item));
    }

    THROW_CHECK(camera.VerifyParams());
    reconstruction.AddCamera(std::move(camera));
  }
}

void ReadCamerasText(Reconstruction& reconstruction, const std::string& path) {
  std::ifstream file(path);
  THROW_CHECK_FILE_OPEN(file, path);
  ReadCamerasText(reconstruction, file);
}

void ReadFramesText(Reconstruction& reconstruction, std::istream& stream) {
  THROW_CHECK(stream.good());

  std::string line;
  std::string item;

  while (std::getline(stream, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    Frame frame;

    // ID
    std::getline(line_stream, item, ' ');
    frame.SetFrameId(std::stoul(item));

    // RIG_ID
    std::getline(line_stream, item, ' ');
    frame.SetRigId(std::stoul(item));

    // RIG_FROM_WORLD

    Rigid3d rig_from_world;

    std::getline(line_stream, item, ' ');
    rig_from_world.rotation.w() = std::stold(item);

    std::getline(line_stream, item, ' ');
    rig_from_world.rotation.x() = std::stold(item);

    std::getline(line_stream, item, ' ');
    rig_from_world.rotation.y() = std::stold(item);

    std::getline(line_stream, item, ' ');
    rig_from_world.rotation.z() = std::stold(item);

    std::getline(line_stream, item, ' ');
    rig_from_world.translation.x() = std::stold(item);

    std::getline(line_stream, item, ' ');
    rig_from_world.translation.y() = std::stold(item);

    std::getline(line_stream, item, ' ');
    rig_from_world.translation.z() = std::stold(item);

    frame.SetRigFromWorld(rig_from_world);

    // DATA_IDS
    std::getline(line_stream, item, ' ');
    const uint32_t num_data_ids = std::stoul(item);
    for (uint32_t i = 0; i < num_data_ids; ++i) {
      data_t data_id;
      std::getline(line_stream, item, ' ');
      data_id.sensor_id.type = SensorTypeFromString(item);
      std::getline(line_stream, item, ' ');
      data_id.sensor_id.id = std::stoul(item);
      std::getline(line_stream, item, ' ');
      data_id.id = std::stoull(item);
      frame.AddDataId(data_id);
    }

    reconstruction.AddFrame(std::move(frame));
  }
}

void ReadFramesText(Reconstruction& reconstruction, const std::string& path) {
  std::ifstream file(path);
  THROW_CHECK_FILE_OPEN(file, path);
  ReadFramesText(reconstruction, file);
}

void ReadImagesText(Reconstruction& reconstruction, std::istream& stream) {
  THROW_CHECK(stream.good());

  // Handle backwards-compatibility for when we didn't have rigs and frames.
  const bool is_legacy_reconstruction =
      reconstruction.NumRigs() == 0 && reconstruction.NumFrames() == 0;
  if (is_legacy_reconstruction) {
    CreateOneRigPerCamera(reconstruction);
  }

  const std::unordered_map<image_t, Frame*> image_to_frame =
      ExtractImageToFramePtr(reconstruction);

  std::string line;
  std::string item;

  std::vector<Eigen::Vector2d> points2D;
  std::vector<point3D_t> point3D_ids;

  while (std::getline(stream, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');
    const image_t image_id = std::stoul(item);

    class Image image;
    image.SetImageId(image_id);

    Rigid3d cam_from_world;

    std::getline(line_stream1, item, ' ');
    cam_from_world.rotation.w() = std::stold(item);

    std::getline(line_stream1, item, ' ');
    cam_from_world.rotation.x() = std::stold(item);

    std::getline(line_stream1, item, ' ');
    cam_from_world.rotation.y() = std::stold(item);

    std::getline(line_stream1, item, ' ');
    cam_from_world.rotation.z() = std::stold(item);

    std::getline(line_stream1, item, ' ');
    cam_from_world.translation.x() = std::stold(item);

    std::getline(line_stream1, item, ' ');
    cam_from_world.translation.y() = std::stold(item);

    std::getline(line_stream1, item, ' ');
    cam_from_world.translation.z() = std::stold(item);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    image.SetCameraId(std::stoul(item));

    if (is_legacy_reconstruction) {
      CreateFrameForImage(image, cam_from_world, reconstruction);
      image.SetFrameId(image.ImageId());
      image.SetFramePtr(&reconstruction.Frame(image.ImageId()));
    } else {
      Frame* frame = image_to_frame.at(image.ImageId());
      image.SetFrameId(frame->FrameId());
      image.SetFramePtr(frame);
    }

    // NAME
    std::getline(line_stream1, item, ' ');
    image.SetName(item);

    // POINTS2D
    if (!std::getline(stream, line)) {
      break;
    }

    StringTrim(&line);
    std::stringstream line_stream2(line);

    points2D.clear();
    point3D_ids.clear();

    if (!line.empty()) {
      while (!line_stream2.eof()) {
        Eigen::Vector2d point;

        std::getline(line_stream2, item, ' ');
        point.x() = std::stold(item);

        std::getline(line_stream2, item, ' ');
        point.y() = std::stold(item);

        points2D.push_back(point);

        std::getline(line_stream2, item, ' ');
        if (item == "-1") {
          point3D_ids.push_back(kInvalidPoint3DId);
        } else {
          point3D_ids.push_back(std::stoll(item));
        }
      }
    }

    image.SetPoints2D(points2D);

    for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
         ++point2D_idx) {
      if (point3D_ids[point2D_idx] != kInvalidPoint3DId) {
        image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
      }
    }

    reconstruction.AddImage(std::move(image));
  }
}

void ReadImagesText(Reconstruction& reconstruction, const std::string& path) {
  std::ifstream file(path);
  THROW_CHECK_FILE_OPEN(file, path);
  ReadImagesText(reconstruction, file);
}

void ReadPoints3DText(Reconstruction& reconstruction, std::istream& stream) {
  THROW_CHECK(stream.good());

  std::string line;
  std::string item;

  while (std::getline(stream, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    // ID
    std::getline(line_stream, item, ' ');
    const point3D_t point3D_id = std::stoll(item);

    struct Point3D point3D;

    // XYZ
    std::getline(line_stream, item, ' ');
    point3D.xyz(0) = std::stold(item);

    std::getline(line_stream, item, ' ');
    point3D.xyz(1) = std::stold(item);

    std::getline(line_stream, item, ' ');
    point3D.xyz(2) = std::stold(item);

    // Color
    std::getline(line_stream, item, ' ');
    point3D.color(0) = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    point3D.color(1) = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    point3D.color(2) = static_cast<uint8_t>(std::stoi(item));

    // ERROR
    std::getline(line_stream, item, ' ');
    point3D.error = std::stold(item);

    // TRACK
    while (!line_stream.eof()) {
      TrackElement track_el;

      std::getline(line_stream, item, ' ');
      StringTrim(&item);
      if (item.empty()) {
        break;
      }
      track_el.image_id = std::stoul(item);

      std::getline(line_stream, item, ' ');
      track_el.point2D_idx = std::stoul(item);

      point3D.track.AddElement(track_el);
    }

    point3D.track.Compress();

    reconstruction.AddPoint3D(point3D_id, std::move(point3D));
  }
}

void ReadPoints3DText(Reconstruction& reconstruction, const std::string& path) {
  std::ifstream file(path);
  THROW_CHECK_FILE_OPEN(file, path);
  ReadPoints3DText(reconstruction, file);
}

void WriteRigsText(const Reconstruction& reconstruction, std::ostream& stream) {
  THROW_CHECK(stream.good());

  // Ensure that we don't loose any precision by storing in text.
  stream.precision(17);

  stream << "# Rig calib list with one line of data per calib:\n";
  stream << "#   RIG_ID, NUM_SENSORS, REF_SENSOR_TYPE, REF_SENSOR_ID, "
            "SENSORS[] as (SENSOR_TYPE, SENSOR_ID, HAS_POSE, [QW, QX, QY, QZ, "
            "TX, TY, TZ])\n";
  stream << "# Number of rigs: " << reconstruction.NumRigs() << '\n';

  for (const camera_t rig_id : ExtractSortedIds(reconstruction.Rigs())) {
    const Rig& rig = reconstruction.Rig(rig_id);

    std::ostringstream line;
    line.precision(17);

    line << rig_id << " ";

    line << rig.NumSensors() << " ";

    if (rig.NumSensors() > 0) {
      const sensor_t ref_sensor_id = rig.RefSensorId();
      line << ref_sensor_id.type << " ";
      line << ref_sensor_id.id << " ";
    }

    for (const auto& [sensor_id, sensor_from_rig] : rig.Sensors()) {
      line << sensor_id.type << " ";
      line << sensor_id.id << " ";
      if (sensor_from_rig.has_value()) {
        line << "1 ";
        line << sensor_from_rig->rotation.w() << " ";
        line << sensor_from_rig->rotation.x() << " ";
        line << sensor_from_rig->rotation.y() << " ";
        line << sensor_from_rig->rotation.z() << " ";
        line << sensor_from_rig->translation.x() << " ";
        line << sensor_from_rig->translation.y() << " ";
        line << sensor_from_rig->translation.z() << " ";
      } else {
        line << "0 ";
      }
    }

    std::string line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);

    stream << line_string << '\n';
  }
}

void WriteRigsText(const Reconstruction& reconstruction,
                   const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  THROW_CHECK_FILE_OPEN(file, path);
  WriteRigsText(reconstruction, file);
}

void WriteCamerasText(const Reconstruction& reconstruction,
                      std::ostream& stream) {
  THROW_CHECK(stream.good());

  // Ensure that we don't loose any precision by storing in text.
  stream.precision(17);

  stream << "# Camera list with one line of data per camera:\n";
  stream << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
  stream << "# Number of cameras: " << reconstruction.NumCameras() << '\n';

  for (const camera_t camera_id : ExtractSortedIds(reconstruction.Cameras())) {
    const Camera& camera = reconstruction.Camera(camera_id);

    std::ostringstream line;
    line.precision(17);

    line << camera_id << " ";
    line << camera.ModelName() << " ";
    line << camera.width << " ";
    line << camera.height << " ";

    for (const double param : camera.params) {
      line << param << " ";
    }

    std::string line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);

    stream << line_string << '\n';
  }
}

void WriteCamerasText(const Reconstruction& reconstruction,
                      const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  THROW_CHECK_FILE_OPEN(file, path);
  WriteCamerasText(reconstruction, file);
}

void WriteFramesText(const Reconstruction& reconstruction,
                     std::ostream& stream) {
  THROW_CHECK(stream.good());

  const std::vector<frame_t> frame_ids = ExtractSortedIds<frame_t, Frame>(
      reconstruction.Frames(),
      [](const Frame& frame) { return frame.HasPose(); });

  // Ensure that we don't loose any precision by storing in text.
  stream.precision(17);

  stream << "# Frame list with one line of data per frame:\n";
  stream << "#   FRAME_ID, RIG_ID, "
            "RIG_FROM_WORLD[QW, QX, QY, QZ, TX, TY, TZ], NUM_DATA_IDS, "
            "DATA_IDS[] as (SENSOR_TYPE, SENSOR_ID, DATA_ID)\n";
  stream << "# Number of frames: " << frame_ids.size() << '\n';

  stream.precision(17);

  for (const frame_t frame_id : frame_ids) {
    const Frame& frame = reconstruction.Frame(frame_id);

    stream << frame_id << " ";
    stream << frame.RigId() << " ";

    const Rigid3d& rig_from_world = frame.RigFromWorld();
    stream << rig_from_world.rotation.w() << " ";
    stream << rig_from_world.rotation.x() << " ";
    stream << rig_from_world.rotation.y() << " ";
    stream << rig_from_world.rotation.z() << " ";
    stream << rig_from_world.translation.x() << " ";
    stream << rig_from_world.translation.y() << " ";
    stream << rig_from_world.translation.z() << " ";

    const std::set<data_t>& data_ids = frame.DataIds();
    stream << data_ids.size();
    for (const data_t& data_id : data_ids) {
      stream << " " << data_id.sensor_id.type << " " << data_id.sensor_id.id
             << " " << data_id.id;
    }

    stream << '\n';
  }
}

void WriteFramesText(const Reconstruction& reconstruction,
                     const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  THROW_CHECK_FILE_OPEN(file, path);
  WriteFramesText(reconstruction, file);
}

void WriteImagesText(const Reconstruction& reconstruction,
                     std::ostream& stream) {
  THROW_CHECK(stream.good());

  // Ensure that we don't loose any precision by storing in text.
  stream.precision(17);

  stream << "# Image list with two lines of data per image:\n";
  stream << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, "
            "NAME\n";
  stream << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
  stream << "# Number of images: " << reconstruction.NumRegImages()
         << ", mean observations per image: "
         << reconstruction.ComputeMeanObservationsPerRegImage() << '\n';

  std::ostringstream line;
  line.precision(17);

  for (const image_t image_id : reconstruction.RegImageIds()) {
    const Image& image = reconstruction.Image(image_id);

    line.str("");
    line.clear();

    line << image_id << " ";

    const Rigid3d& cam_from_world = image.CamFromWorld();
    line << cam_from_world.rotation.w() << " ";
    line << cam_from_world.rotation.x() << " ";
    line << cam_from_world.rotation.y() << " ";
    line << cam_from_world.rotation.z() << " ";
    line << cam_from_world.translation.x() << " ";
    line << cam_from_world.translation.y() << " ";
    line << cam_from_world.translation.z() << " ";

    line << image.CameraId() << " ";

    line << image.Name();

    stream << line.str() << '\n';

    line.str("");
    line.clear();

    for (const Point2D& point2D : image.Points2D()) {
      line << point2D.xy(0) << " ";
      line << point2D.xy(1) << " ";
      if (point2D.HasPoint3D()) {
        line << point2D.point3D_id << " ";
      } else {
        line << -1 << " ";
      }
    }
    if (image.NumPoints2D() > 0) {
      line.seekp(-1, std::ios_base::end);
    }
    stream << line.str() << '\n';
  }
}

void WriteImagesText(const Reconstruction& reconstruction,
                     const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  THROW_CHECK_FILE_OPEN(file, path);
  WriteImagesText(reconstruction, file);
}

void WritePoints3DText(const Reconstruction& reconstruction,
                       std::ostream& stream) {
  THROW_CHECK(stream.good());

  // Ensure that we don't loose any precision by storing in text.
  stream.precision(17);

  stream << "# 3D point list with one line of data per point:\n";
  stream << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, "
            "TRACK[] as (IMAGE_ID, POINT2D_IDX)\n";
  stream << "# Number of points: " << reconstruction.NumPoints3D()
         << ", mean track length: " << reconstruction.ComputeMeanTrackLength()
         << '\n';

  for (const point3D_t point3D_id :
       ExtractSortedIds(reconstruction.Points3D())) {
    const Point3D& point3D = reconstruction.Point3D(point3D_id);

    stream << point3D_id << " ";
    stream << point3D.xyz(0) << " ";
    stream << point3D.xyz(1) << " ";
    stream << point3D.xyz(2) << " ";
    stream << static_cast<int>(point3D.color(0)) << " ";
    stream << static_cast<int>(point3D.color(1)) << " ";
    stream << static_cast<int>(point3D.color(2)) << " ";
    stream << point3D.error << " ";

    std::ostringstream line;
    line.precision(17);

    for (const auto& track_el : point3D.track.Elements()) {
      line << track_el.image_id << " ";
      line << track_el.point2D_idx << " ";
    }

    std::string line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);

    stream << line_string << '\n';
  }
}

void WritePoints3DText(const Reconstruction& reconstruction,
                       const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  THROW_CHECK_FILE_OPEN(file, path);
  WritePoints3DText(reconstruction, file);
}

// Per-FRAME (rig) 3x3 position covariance matrices.
// One line per frame: FRAME_ID RIG_ID  COV_POS[3x3 row-major]
void WritePositionCovariancesText(const Reconstruction& reconstruction,
                                  std::ostream& stream) {
  THROW_CHECK(stream.good());

  // 1) Frames with a pose
  const std::vector<frame_t> frame_ids = ExtractSortedIds<frame_t, Frame>(
      reconstruction.Frames(),
      [](const Frame& f) { return f.HasPose(); });

  // Header
  stream.precision(17);
  stream << "# Position covariance list with one line of data per frame:\n";
  stream << "#   FRAME_ID, RIG_ID, COV_POS[3x3 row-major]\n";
  stream << "#   Position covariance is in world units (3x3).\n";
  stream << "# Number of frames: " << frame_ids.size() << '\n';
  if (frame_ids.empty()) return;

  auto write_nan9 = [&](std::ostream& out) {
    for (int i = 0; i < 9; ++i) out << "nan" << (i == 8 ? "" : " ");
  };
  auto write3x3 = [&](const Eigen::Matrix3d& M, std::ostream& out) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        out << M(r, c) << " ";
  };

  // 2) Build the BA problem used for export (no solve)
  BundleAdjustmentOptions ba_opts;
  BundleAdjustmentConfig ba_cfg;
  for (const frame_t fid : reconstruction.RegFrameIds()) {
    const Frame& fr = reconstruction.Frame(fid);
    for (const data_t& did : fr.ImageIds()) ba_cfg.AddImage(did.id);
  }

  if (ba_cfg.NumImages() < 2) {
    for (const frame_t fid : frame_ids) {
      const Frame& fr = reconstruction.Frame(fid);
      std::ostringstream line; line.precision(17);
      line << fid << " " << fr.RigId() << " ";
      write_nan9(line);
      std::string s = line.str(); if (!s.empty() && s.back() == ' ') s.pop_back();
      stream << s << '\n';
    }
    return;
  }

  // Fix gauge similarly to global BA
  ba_cfg.FixGauge(BundleAdjustmentGauge::TWO_CAMS_FROM_WORLD);

  // Create BA (needs non-const reconstruction)
  Reconstruction& nc = const_cast<Reconstruction&>(reconstruction);
  std::unique_ptr<BundleAdjuster> ba =
      CreateDefaultBundleAdjuster(std::move(ba_opts), ba_cfg, nc);
  ceres::Problem& problem = *ba->Problem();

  // 3) Collect exact ceres parameter pointers for each frame's rig pose
  struct PosePtrs { const double* q=nullptr; const double* t=nullptr; image_t key=kInvalidImageId; };
  std::vector<PosePtrs> ptrs(frame_ids.size());

  for (size_t i = 0; i < frame_ids.size(); ++i) {
    const frame_t fid = frame_ids[i];
    const Frame& fr = reconstruction.Frame(fid);

    // Pick any registered image in the frame as a key tag (not used below, but handy if you log)
    for (const auto& did : fr.ImageIds()) { ptrs[i].key = did.id; break; }

    const Rigid3d& T_rw = fr.RigFromWorld();

    // Prefer project-specific raw buffers if your fork exposes them:
    // if (fr.HasQvecData() && problem.HasParameterBlock(fr.QvecData()))
    //   ptrs[i].q = fr.QvecData(); // [qw qx qy qz]
    // if (fr.HasTvecData() && problem.HasParameterBlock(fr.TvecData()))
    //   ptrs[i].t = fr.TvecData(); // [tx ty tz]

    // Fallback to Eigen buffers (verify they are actual parameter blocks):
    if (!ptrs[i].q) {
      const double* q_xyzw = T_rw.rotation.coeffs().data(); // [x y z w]
      if (q_xyzw && problem.HasParameterBlock(q_xyzw)) ptrs[i].q = q_xyzw;
    }
    if (!ptrs[i].t) {
      const double* t_eig  = T_rw.translation.data();       // [tx ty tz]
      if (t_eig  && problem.HasParameterBlock(t_eig))  ptrs[i].t = t_eig;
    }
  }

  // 4) Stabilize the reduced system:
  //    - Hold ALL rig rotations constant (we only want translation covariance)
  //    - Fix one frame's translation as an anchor to kill global translation gauge
  //      (choose the first frame for which we found a t-block)
  {
    const double* anchor_t = nullptr;
    for (size_t i = 0; i < ptrs.size(); ++i) {
      if (ptrs[i].q && problem.HasParameterBlock(ptrs[i].q)) {
        problem.SetParameterBlockConstant(const_cast<double*>(ptrs[i].q));
      }
      if (!anchor_t && ptrs[i].t && problem.HasParameterBlock(ptrs[i].t)) {
        anchor_t = ptrs[i].t;
      }
    }
    if (anchor_t) {
      problem.SetParameterBlockConstant(const_cast<double*>(anchor_t));
    }
  }

  // 5) Build the custom pose list (only those frames with both q and t pointers;
  //    q will be constant, t is variable except the anchor we just fixed).
  std::vector<colmap::internal::PoseParam> custom_poses;
  custom_poses.reserve(frame_ids.size());
  for (size_t i = 0; i < frame_ids.size(); ++i) {
    if (!ptrs[i].q || !ptrs[i].t) continue;
    colmap::internal::PoseParam p;
    p.image_id = (ptrs[i].key == kInvalidImageId) ? static_cast<image_t>(-(long long)i-1) : ptrs[i].key;
    p.qvec = ptrs[i].q;
    p.tvec = ptrs[i].t;
    custom_poses.push_back(p);
  }

  // 6) Try BACovariance (pose-only Schur) with increasing damping
  auto dump_with = [&](const BACovariance& cov) {
    for (size_t i = 0; i < frame_ids.size(); ++i) {
      const frame_t fid = frame_ids[i];
      const Frame& fr = reconstruction.Frame(fid);
      std::ostringstream line; line.precision(17);
      line << fid << " " << fr.RigId() << " ";

      const double* t_ptr = ptrs[i].t;
      if (!t_ptr) {
        write_nan9(line);
      } else {
        // Directly fetch the 3x3 covariance for the translation block
        std::optional<Eigen::MatrixXd> Ctt = cov.GetOtherParamsCov(t_ptr);
        if (!Ctt || Ctt->rows() != 3 || Ctt->cols() != 3) {
          write_nan9(line);
        } else {
          write3x3(Ctt->cast<double>(), line);
        }
      }

      std::string s = line.str(); if (!s.empty() && s.back() == ' ') s.pop_back();
      stream << s << '\n';
    }
  };

  const double lambdas[] = {1e-2, 1e-1, 1.0, 10.0, 1e2, 1e3, 1e4};
  for (double lambda : lambdas) {
    BACovarianceOptions cov_opts;
    cov_opts.params = BACovarianceOptions::Params::POSES;
    cov_opts.damping = lambda;
    cov_opts.experimental_custom_poses = custom_poses; // copy each try

    std::optional<BACovariance> cov = EstimateBACovariance(cov_opts, reconstruction, *ba);
    if (cov) {
      dump_with(*cov);
      return;
    }
  }

  // If every attempt failed, write NaNs for all frames.
  for (const frame_t fid : frame_ids) {
    const Frame& fr = reconstruction.Frame(fid);
    std::ostringstream line; line.precision(17);
    line << fid << " " << fr.RigId() << " ";
    write_nan9(line);
    std::string s = line.str(); if (!s.empty() && s.back() == ' ') s.pop_back();
    stream << s << '\n';
  }
}



void WritePositionCovariancesText(const Reconstruction& reconstruction,
                              const std::string& path) {
  std::ofstream file(path, std::ios::trunc);
  THROW_CHECK_FILE_OPEN(file, path);
  WritePositionCovariancesText(reconstruction, file);
}

}  // namespace colmap
