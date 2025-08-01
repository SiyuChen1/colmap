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

#pragma once

#include "colmap/feature/types.h"
#include "colmap/geometry/pose_prior.h"
#include "colmap/scene/camera.h"
#include "colmap/scene/image.h"
#include "colmap/scene/two_view_geometry.h"
#include "colmap/sensor/rig.h"
#include "colmap/util/eigen_alignment.h"
#include "colmap/util/types.h"

#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <sqlite3.h>

namespace colmap {

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    FeatureKeypointsBlob;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    FeatureDescriptorsBlob;
typedef Eigen::Matrix<point2D_t, Eigen::Dynamic, 2, Eigen::RowMajor>
    FeatureMatchesBlob;

// Database class to read and write images, features, cameras, matches, etc.
// from a SQLite database. The class is not thread-safe and must not be accessed
// concurrently. The class is optimized for single-thread speed and for optimal
// performance, wrap multiple method calls inside a leading `BeginTransaction`
// and trailing `EndTransaction`.
class Database {
 public:
  const static int kSchemaVersion = 1;

  // The maximum number of images, that can be stored in the database.
  // This limitation arises due to the fact, that we generate unique IDs for
  // image pairs manually. Note: do not change this to
  // another type than `size_t`.
  const static size_t kMaxNumImages;

  // Can be used to construct temporary in-memory database.
  const static std::string kInMemoryDatabasePath;

  Database();
  explicit Database(const std::string& path);
  ~Database();

  // Open and close database. The same database should not be opened
  // concurrently in multiple threads or processes.
  //
  // On Windows, the input path is converted from the local code page to UTF-8
  // for compatibility with SQLite. On POSIX platforms, the path is assumed to
  // be UTF-8.
  void Open(const std::string& path);
  void Close();

  // Check if entry already exists in database. For image pairs, the order of
  // `image_id1` and `image_id2` does not matter.
  bool ExistsRig(rig_t rig_id) const;
  bool ExistsCamera(camera_t camera_id) const;
  bool ExistsFrame(frame_t frame_id) const;
  bool ExistsImage(image_t image_id) const;
  bool ExistsImageWithName(const std::string& name) const;
  bool ExistsPosePrior(image_t image_id) const;
  bool ExistsKeypoints(image_t image_id) const;
  bool ExistsDescriptors(image_t image_id) const;
  bool ExistsMatches(image_t image_id1, image_t image_id2) const;
  bool ExistsInlierMatches(image_t image_id1, image_t image_id2) const;

  // Number of rows in `rigs` table.
  size_t NumRigs() const;

  // Number of rows in `cameras` table.
  size_t NumCameras() const;

  //  Number of rows in `frames` table.
  size_t NumFrames() const;

  //  Number of rows in `images` table.
  size_t NumImages() const;

  //  Number of rows in `pose_priors` table.
  size_t NumPosePriors() const;

  // Sum of `rows` column in `keypoints` table, i.e. number of total keypoints.
  size_t NumKeypoints() const;

  // The number of keypoints for the image with most features.
  size_t MaxNumKeypoints() const;

  // Number of descriptors for specific image.
  size_t NumKeypointsForImage(image_t image_id) const;

  // Sum of `rows` column in `descriptors` table,
  // i.e. number of total descriptors.
  size_t NumDescriptors() const;

  // The number of descriptors for the image with most features.
  size_t MaxNumDescriptors() const;

  // Number of descriptors for specific image.
  size_t NumDescriptorsForImage(image_t image_id) const;

  // Sum of `rows` column in `matches` table, i.e. number of total matches.
  size_t NumMatches() const;

  // Sum of `rows` column in `two_view_geometries` table,
  // i.e. number of total inlier matches.
  size_t NumInlierMatches() const;

  // Number of rows in `matches` table.
  size_t NumMatchedImagePairs() const;

  // Number of rows in `two_view_geometries` table.
  size_t NumVerifiedImagePairs() const;

  // Each image pair is assigned an unique ID in the `matches` and
  // `two_view_geometries` table. We intentionally avoid to store the pairs in a
  // separate table by using e.g. AUTOINCREMENT, since the overhead of querying
  // the unique pair ID is significant.
  inline static image_pair_t ImagePairToPairId(image_t image_id1,
                                               image_t image_id2);

  inline static std::pair<image_t, image_t> PairIdToImagePair(
      image_pair_t pair_id);

  // Return true if image pairs should be swapped. Used to enforce a specific
  // image order to generate unique image pair identifiers independent of the
  // order in which the image identifiers are used.
  inline static bool SwapImagePair(image_t image_id1, image_t image_id2);

  // Read an existing entry in the database. The user is responsible for making
  // sure that the entry actually exists. For image pairs, the order of
  // `image_id1` and `image_id2` does not matter.

  Rig ReadRig(rig_t rig_id) const;
  std::optional<Rig> ReadRigWithSensor(sensor_t sensor_id) const;
  std::vector<Rig> ReadAllRigs() const;

  Camera ReadCamera(camera_t camera_id) const;
  std::vector<Camera> ReadAllCameras() const;

  Frame ReadFrame(frame_t frame_id) const;
  std::vector<Frame> ReadAllFrames() const;

  Image ReadImage(image_t image_id) const;
  std::optional<Image> ReadImageWithName(const std::string& name) const;
  std::vector<Image> ReadAllImages() const;

  PosePrior ReadPosePrior(image_t image_id) const;

  FeatureKeypointsBlob ReadKeypointsBlob(image_t image_id) const;
  FeatureKeypoints ReadKeypoints(image_t image_id) const;
  FeatureDescriptors ReadDescriptors(image_t image_id) const;

  FeatureMatchesBlob ReadMatchesBlob(image_t image_id1,
                                     image_t image_id2) const;
  FeatureMatches ReadMatches(image_t image_id1, image_t image_id2) const;
  std::vector<std::pair<image_pair_t, FeatureMatchesBlob>> ReadAllMatchesBlob()
      const;
  std::vector<std::pair<image_pair_t, FeatureMatches>> ReadAllMatches() const;

  TwoViewGeometry ReadTwoViewGeometry(image_t image_id1,
                                      image_t image_id2) const;
  std::vector<std::pair<image_pair_t, TwoViewGeometry>> ReadTwoViewGeometries()
      const;

  // Read all image pairs that have an entry in the `two_view_geometry`
  // table with at least one inlier match and their number of inlier matches.
  std::vector<std::pair<image_pair_t, int>> ReadTwoViewGeometryNumInliers()
      const;

  // Add new rig and return its database identifier. If `use_rig_id`
  // is false a new identifier is automatically generated.
  rig_t WriteRig(const Rig& rig, bool use_rig_id = false) const;

  // Add new camera and return its database identifier. If `use_camera_id`
  // is false a new identifier is automatically generated.
  camera_t WriteCamera(const Camera& camera, bool use_camera_id = false) const;

  // Add new frame and return its database identifier. If `use_frame_id`
  // is false a new identifier is automatically generated.
  frame_t WriteFrame(const Frame& frame, bool use_frame_id = false) const;

  // Add new image and return its database identifier. If `use_image_id`
  // is false a new identifier is automatically generated.
  image_t WriteImage(const Image& image, bool use_image_id = false) const;

  // Write a new entry in the database. The user is responsible for making sure
  // that the entry does not yet exist. For image pairs, the order of
  // `image_id1` and `image_id2` does not matter.
  void WritePosePrior(image_t image_id, const PosePrior& pose_prior) const;
  void WriteKeypoints(image_t image_id,
                      const FeatureKeypoints& keypoints) const;
  void WriteKeypoints(image_t image_id, const FeatureKeypointsBlob& blob) const;
  void WriteDescriptors(image_t image_id,
                        const FeatureDescriptors& descriptors) const;
  void WriteMatches(image_t image_id1,
                    image_t image_id2,
                    const FeatureMatches& matches) const;
  void WriteMatches(image_t image_id1,
                    image_t image_id2,
                    const FeatureMatchesBlob& blob) const;
  void WriteTwoViewGeometry(image_t image_id1,
                            image_t image_id2,
                            const TwoViewGeometry& two_view_geometry) const;

  // Update an existing rig in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdateRig(const Rig& rig) const;

  // Update an existing camera in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdateCamera(const Camera& camera) const;

  // Update an existing frame in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdateFrame(const Frame& frame) const;

  // Update an existing image in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdateImage(const Image& image) const;

  // Update an existing pose_prior in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdatePosePrior(image_t image_id, const PosePrior& pose_prior) const;

  // Delete matches of an image pair.
  void DeleteMatches(image_t image_id1, image_t image_id2) const;

  // Delete inlier matches of an image pair.
  void DeleteInlierMatches(image_t image_id1, image_t image_id2) const;

  // Clear all database tables
  void ClearAllTables() const;

  // Clear the entire rigs table
  void ClearRigs() const;

  // Clear the entire cameras table
  void ClearCameras() const;

  // Clear the entire frames table
  void ClearFrames() const;

  // Clear the entire images, keypoints, and descriptors tables
  void ClearImages() const;

  // Clear the entire pose_priors table
  void ClearPosePriors() const;

  // Clear the entire descriptors table
  void ClearDescriptors() const;

  // Clear the entire keypoints table
  void ClearKeypoints() const;

  // Clear the entire matches table.
  void ClearMatches() const;

  // Clear the entire inlier matches table.
  void ClearTwoViewGeometries() const;

  // Merge two databases into a single, new database.
  static void Merge(const Database& database1,
                    const Database& database2,
                    Database* merged_database);

 private:
  friend class DatabaseTransaction;

  // Combine multiple queries into one transaction by wrapping a code section
  // into a `BeginTransaction` and `EndTransaction`. You can create a scoped
  // transaction with `DatabaseTransaction` that ends when the transaction
  // object is destructed. Combining queries results in faster transaction time
  // due to reduced locking of the database etc.
  void BeginTransaction() const;
  void EndTransaction() const;

  // Prepare SQL statements once at construction of the database, and reuse
  // the statements for multiple queries by resetting their states.
  void PrepareSQLStatements();
  void FinalizeSQLStatements();

  // Create database tables, if not existing, called when opening a database.
  void CreateTables() const;
  void CreateRigTable() const;
  void CreateRigSensorsTable() const;
  void CreateCameraTable() const;
  void CreateFrameTable() const;
  void CreateFrameDataTable() const;
  void CreateImageTable() const;
  void CreatePosePriorTable() const;
  void CreateKeypointsTable() const;
  void CreateDescriptorsTable() const;
  void CreateMatchesTable() const;
  void CreateTwoViewGeometriesTable() const;

  void UpdateSchema() const;

  bool ExistsTable(const std::string& table_name) const;
  bool ExistsColumn(const std::string& table_name,
                    const std::string& column_name) const;

  bool ExistsRowId(sqlite3_stmt* sql_stmt, sqlite3_int64 row_id) const;
  bool ExistsRowString(sqlite3_stmt* sql_stmt,
                       const std::string& row_entry) const;

  size_t CountRows(const std::string& table) const;
  size_t CountRowsForEntry(sqlite3_stmt* sql_stmt, sqlite3_int64 row_id) const;
  size_t SumColumn(const std::string& column, const std::string& table) const;
  size_t MaxColumn(const std::string& column, const std::string& table) const;

  sqlite3* database_ = nullptr;

  // Check if elements got removed from the database to only apply
  // the VACUUM command in such case
  mutable bool database_entry_deleted_ = false;

  // Ensure that only one database object at a time updates the schema of a
  // database. Since the schema is updated every time a database is opened, this
  // is to ensure that there are no race conditions ("database locked" error
  // messages) when the user actually only intends to read from the database,
  // which requires to open it.
  static std::mutex update_schema_mutex_;

  // Used to ensure that only one transaction is active at the same time.
  std::mutex transaction_mutex_;

  // A collection of all `sqlite3_stmt` objects for deletion in the destructor.
  std::vector<sqlite3_stmt*> sql_stmts_;

  // num_*
  sqlite3_stmt* sql_stmt_num_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_num_descriptors_ = nullptr;

  // exists_*
  sqlite3_stmt* sql_stmt_exists_rig_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_frame_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_image_id_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_image_name_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_pose_prior_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_two_view_geometry_ = nullptr;

  // add_*
  sqlite3_stmt* sql_stmt_add_rig_ = nullptr;
  sqlite3_stmt* sql_stmt_add_rig_sensor_ = nullptr;
  sqlite3_stmt* sql_stmt_add_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_add_frame_ = nullptr;
  sqlite3_stmt* sql_stmt_add_frame_data_ = nullptr;
  sqlite3_stmt* sql_stmt_add_image_ = nullptr;

  // update_*
  sqlite3_stmt* sql_stmt_update_rig_ = nullptr;
  sqlite3_stmt* sql_stmt_update_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_update_frame_ = nullptr;
  sqlite3_stmt* sql_stmt_update_image_ = nullptr;
  sqlite3_stmt* sql_stmt_update_pose_prior_ = nullptr;

  // read_*
  sqlite3_stmt* sql_stmt_read_rig_ = nullptr;
  sqlite3_stmt* sql_stmt_read_rigs_ = nullptr;
  sqlite3_stmt* sql_stmt_read_rig_with_sensor_ = nullptr;
  sqlite3_stmt* sql_stmt_read_rig_with_ref_sensor_ = nullptr;
  sqlite3_stmt* sql_stmt_read_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_read_cameras_ = nullptr;
  sqlite3_stmt* sql_stmt_read_frame_ = nullptr;
  sqlite3_stmt* sql_stmt_read_frames_ = nullptr;
  sqlite3_stmt* sql_stmt_read_image_id_ = nullptr;
  sqlite3_stmt* sql_stmt_read_image_with_name_ = nullptr;
  sqlite3_stmt* sql_stmt_read_images_ = nullptr;
  sqlite3_stmt* sql_stmt_read_pose_prior_ = nullptr;
  sqlite3_stmt* sql_stmt_read_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_read_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_read_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_read_matches_all_ = nullptr;
  sqlite3_stmt* sql_stmt_read_two_view_geometry_ = nullptr;
  sqlite3_stmt* sql_stmt_read_two_view_geometries_ = nullptr;
  sqlite3_stmt* sql_stmt_read_two_view_geometry_num_inliers_ = nullptr;

  // write_*
  sqlite3_stmt* sql_stmt_write_pose_prior_ = nullptr;
  sqlite3_stmt* sql_stmt_write_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_write_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_write_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_write_two_view_geometry_ = nullptr;

  // delete_*
  sqlite3_stmt* sql_stmt_delete_rig_sensors_ = nullptr;
  sqlite3_stmt* sql_stmt_delete_frame_data_ = nullptr;
  sqlite3_stmt* sql_stmt_delete_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_delete_two_view_geometry_ = nullptr;

  // clear_*
  sqlite3_stmt* sql_stmt_clear_rigs_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_cameras_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_frames_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_images_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_pose_priors_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_two_view_geometries_ = nullptr;
};

// This class automatically manages the scope of a database transaction by
// calling `BeginTransaction` and `EndTransaction` during construction and
// destruction, respectively.
class DatabaseTransaction {
 public:
  explicit DatabaseTransaction(Database* database);
  ~DatabaseTransaction();

 private:
  NON_COPYABLE(DatabaseTransaction)
  NON_MOVABLE(DatabaseTransaction)
  Database* database_;
  std::unique_lock<std::mutex> database_lock_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

image_pair_t Database::ImagePairToPairId(const image_t image_id1,
                                         const image_t image_id2) {
  THROW_CHECK_LT(image_id1, kMaxNumImages);
  THROW_CHECK_LT(image_id2, kMaxNumImages);
  if (SwapImagePair(image_id1, image_id2)) {
    return static_cast<image_pair_t>(kMaxNumImages) * image_id2 + image_id1;
  } else {
    return static_cast<image_pair_t>(kMaxNumImages) * image_id1 + image_id2;
  }
}

std::pair<image_t, image_t> Database::PairIdToImagePair(
    const image_pair_t pair_id) {
  const image_t image_id2 = static_cast<image_t>(pair_id % kMaxNumImages);
  const image_t image_id1 =
      static_cast<image_t>((pair_id - image_id2) / kMaxNumImages);
  THROW_CHECK_LT(image_id1, kMaxNumImages);
  THROW_CHECK_LT(image_id2, kMaxNumImages);
  return std::make_pair(image_id1, image_id2);
}

// Return true if image pairs should be swapped. Used to enforce a specific
// image order to generate unique image pair identifiers independent of the
// order in which the image identifiers are used.
bool Database::SwapImagePair(const image_t image_id1, const image_t image_id2) {
  return image_id1 > image_id2;
}

}  // namespace colmap
