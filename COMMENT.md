# Comments of Source Code and Current Implementations
## Geometric verification
## Pose Prior
### Introduction to pose prior
#### What a **pose prior** is here
In this codebase a pose prior is a position prior only (no orientation). It stores:
- 3D position position (either WGS84 lat/lon/alt or Cartesian),
- optional position_covariance (3×3),
- coordinate_system enum.
See `src/colmap/geometry/pose_prior.h`
#### **MSL** or **HAE** altitude in Colmap?
During image import, COLMAP pulls GPS from EXIF via: `Bitmap::ExifLatitude, Bitmap::ExifLongitude, Bitmap::ExifAltitude` defined in `src/colmap/sensor/bitmap.cc` reading the GPSAltitude tag as a simple rational number (no reference, no conversion):
```c++
// bitmap.cc
if (ReadExifTag(handle_.ptr, FIMD_EXIF_GPS, "GPSAltitude", &str)) {
  *altitude = std::stold(result[1]) / std::stold(result[2]);
}
```

There’s no use of GPSAltitudeRef and no geoid/EGM correction.
The reader then stores {lat, lon, alt} into a PosePrior with coordinate_system = WGS84, see `ImageReader::Next(..., PosePrior* pose_prior, ...)`, in `src/colmap/controllers/image_reader.cc`.

Implication: EXIF GPSAltitude is specified as orthometric height (MSL) by the EXIF standard, but COLMAP stores whatever number is there without checking or converting.

When a prior is WGS84, COLMAP converts all stored {lat,lon,alt} to a local Cartesian ENU frame (anchored at the first image with a prior) right after loading the DB: `DatabaseCache::SetupPosePriors()` calls
`GPSTransform::EllipsoidToENU(...)` see `src/colmap/scene/database_cache.cc`.

`GPSTransform::EllipsoidToECEF/ENU` implements the geodetic formulas that assume alt is ellipsoidal height h (HAE) and plugs alt into (N + h) etc, see `src/colmap/geometry/gps.cc (EllipsoidToECEF)`.
```c++
std::vector<Eigen::Vector3d> GPSTransform::EllipsoidToECEF(
    const std::vector<Eigen::Vector3d>& lat_lon_alt) const {
  std::vector<Eigen::Vector3d> xyz_in_ecef(lat_lon_alt.size());

  for (size_t i = 0; i < lat_lon_alt.size(); ++i) {
    const double lat = DegToRad(lat_lon_alt[i](0));
    const double lon = DegToRad(lat_lon_alt[i](1));
    const double alt = lat_lon_alt[i](2);

    const double sin_lat = std::sin(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lat = std::cos(lat);
    const double cos_lon = std::cos(lon);

    // Normalized radius
    const double N = a_ / std::sqrt(1 - e2_ * sin_lat * sin_lat);

    xyz_in_ecef[i](0) = (N + alt) * cos_lat * cos_lon;
    xyz_in_ecef[i](1) = (N + alt) * cos_lat * sin_lon;
    xyz_in_ecef[i](2) = (N * (1 - e2_) + alt) * sin_lat;
  }

  return xyz_in_ecef;
}
// file src/colmap/geometry/gps.cc, line(~118-141)
```

If your inputs are MSL, you should convert to HAE yourself:
$h(\text{HAE}) = H(\text{MSL}) + N(\text{lat,lon})$, where $N$ is the geoid height (EGM96/2008).
##### Additional information about **GPSAltitude** in EXIF 
In EXIF 2.x version, `GPSAltitude (tag 0x0006, key Exif.GPSInfo.GPSAltitude)` is the camera’s recorded altitude from its GPS. It’s stored as a Rational number (numerator/denominator) whose units are meters which should be interpreted together with `GPSAltitudeRef (0x0005)`, which says whether that altitude is above or below sea level:

> GPSAltitudeRef = 0 -> altitude is above mean sea level
> 
> GPSAltitudeRef = 1 -> altitude is below mean sea level (the value is an absolute magnitude)

So the signed altitude in meters is:

> altitude_m = (numerator / denominator) * (+1 if GPSAltitudeRef==0 else -1)

Examples:
> GPSAltitudeRef=0, GPSAltitude=5283/100 -> 52.83 m above sea level
>
> GPSAltitudeRef=1, GPSAltitude=134/1 -> -134 m (134 m below sea level)

In EXIT 3.0 version, released in December 2024, the definition of `GPSAltitudeRef (0x0005)` has been changed. Download link: [Exchangeable image file format for digital still cameras:
Exif Version 3.0](https://ia800401.us.archive.org/11/items/exif-specs-3.0-dc-008-translation-2023-e/EXIF_Specs_3.0_DC-008-Translation-2023-E.pdf).

> Cited its definition on page 92
> 
> Default = 0
>
> 0 = Positive ellipsoidal height (at or above ellipsoidal surface)
>
> 1 = Negative ellipsoidal height (below ellipsoidal surface)
>
> 2 = Positive sea level value (at or above sea level reference)
>
> 3 = Negative sea level value (below sea level reference)
>
> Other = reserved

If GPSAltitudeRef is missing, many tools assume "above sea level."
#### Where a **pose prior** is read
1. From EXIF GPS when you import/extract features
    - `ImageReader::Next(..., PosePrior* pose_prior, ...)` reads EXIF lat/lon/alt and fills a PosePrior in WGS84 if present.
        ```c++
        Eigen::Vector3d position_prior;
            if (bitmap->ExifLatitude(&position_prior.x()) &&
                bitmap->ExifLongitude(&position_prior.y()) &&
                bitmap->ExifAltitude(&position_prior.z())) {
            pose_prior->position = position_prior;
            pose_prior->coordinate_system = PosePrior::CoordinateSystem::WGS84;
            }
        // src/colmap/controllers/image_reader.cc (lines ~300-307).
        ```
    - `FeatureExtractionController` writes that prior into the database table `pose_priors`.
        ```c++
        if (pose_prior.IsValid()) {
            database.WritePosePrior(image.ImageId(), pose_prior);
          }
        // src/colmap/controllers/feature_extraction.cc (~582-586).
        ```
    - DB schema: `CREATE TABLE pose_priors (image_id, position, coordinate_system, position_covariance)` and `Database::WritePosePrior(...)`.
        ```c++
        void Database::CreatePosePriorTable() const {
            const std::string sql =
                "CREATE TABLE IF NOT EXISTS pose_priors"
                "   (image_id                   INTEGER  PRIMARY KEY  NOT NULL,"
                "    position                   BLOB,"
                "    coordinate_system          INTEGER               NOT NULL,"
                "    position_covariance        BLOB,"
                "    FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE "
                "CASCADE);";

            SQLITE3_EXEC(database_, sql.c_str(), nullptr);
            }
        // src/colmap/scene/database.cc (~1842-1853)
        ```
        ```c++
        void Database::WritePosePrior(const image_t image_id,
                                        const PosePrior& pose_prior) const {
            Sqlite3StmtContext context(sql_stmt_write_pose_prior_);

            SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_write_pose_prior_, 1, image_id));
            WriteStaticMatrixBlob(sql_stmt_write_pose_prior_, pose_prior.position, 2);
            SQLITE3_CALL(sqlite3_bind_int64(
                sql_stmt_write_pose_prior_,
                3,
                static_cast<sqlite3_int64>(pose_prior.coordinate_system)));
            WriteStaticMatrixBlob(
                sql_stmt_write_pose_prior_, pose_prior.position_covariance, 4);
            SQLITE3_CALL(sqlite3_step(sql_stmt_write_pose_prior_));
            }
        // src/colmap/scene/database.cc (1030-1044)
        ```
2. From the database into caches/pipelines
    - Matching-time cache loads priors lazily: `FeatureMatcherCache::MaybeLoadPosePriors()` and `GetPosePriorOrNull()`. Defined in `src/colmap/feature/matcher.cc (~298-306, 133-138)`.
    - Incremental SfM loads priors (and, if enabled, converts WGS84 to Cartesian) via `DatabaseCache::SetupPosePriors()`, called from `IncrementalPipeline::LoadDatabase()` when `use_prior_position=true`. Defined in `src/colmap/scene/database_cache.cc (~336-396)` and `src/colmap/controllers/incremental_pipeline.cc (~280-284)`.

    Conversion details

    * For matching, WGS84 is converted to ECEF inside the pairing code (see below).

    * For SfM/BA, if the priors are WGS84 they are converted once to a local ENU Cartesian frame, anchored at the first (lowest id) image with a prior (`GPSTransform::EllipsoidToENU`) meaning the very first image is chosen as reference point for converting, making its ENU all zero. Coordinate system flag is set to CARTESIAN. Defined `src/colmap/scene/database_cache.cc (~360-386)`.

#### Where the prior is used
1. During feature pairing / matching
    
    Priors are used for matching, to choose candidate pairs (not to score descriptors). They are not used inside the actual 2-view geometry estimation/matching cost.
    - Spatial pairing (GPS-based nearest neighbors): SpatialPairGenerator builds a matrix of image positions from priors, converts WGS84 to ECEF, optionally zeros Z (`ignore_z`), subtracts the mean for numerical stability, then does KNN to yield candidate pairs.

        - Implementation: `SpatialPairGenerator::ReadPositionPriorData(...)` and `Next()` defined in `src/colmap/feature/pairing.cc (~660-736)`.
        - Options: 
        `SpatialPairingOptions { ignore_z=true, max_num_neighbors=50, ... }` defined in `src/colmap/feature/pairing.h (~163-170, ~345-353)`.

2. During bundle adjustment (SfM optimization)
    
    Used if and only if you enable `use_prior_position` in the mapper options (or run the `pose_prior_mapper` command). Then the code switches to a special bundle adjuster that uses priors to align and regularize camera positions:
    
    - Global BA path chooses a prior-aware adjuster
        In IncrementalMapper::AdjustGlobalBundle(...) it picks:
        - default BA when priors are disabled, or
        - `CreatePosePriorBundleAdjuster(...)` when `use_prior_position=true` and at least 3 images are registered, see `src/colmap/sfm/incremental_mapper.cc (~805-833)`.
    - Robust Sim(3) alignment to the priors before BA
        - `PosePriorBundleAdjuster` first robustly aligns the current reconstruction to the prior positions with RANSAC over Sim3: `AlignReconstructionToPosePriors(reconstruction_, pose_priors_, ransac_options, &metric_from_orig)` in `src/colmap/estimators/alignment.cc (~240-272)`.
        - The RANSAC max error is either user-specified or derived from the prior covariances ($3\sigma$ heuristic). See in `src/colmap/estimators/bundle_adjustment.cc (~1000-1016)`.

3. Add per-image position prior residuals to the BA problem

    For each image with a valid position covariance, the adjuster adds a residual that penalizes the difference between the camera center and the prior position in world coords: `CovarianceWeightedCostFunctor<AbsolutePosePositionPriorCostFunctor>::Create(prior.position_covariance, normalized_from_metric_*prior.position)` attached to (rotation, translation) parameters of the camera in `src/colmap/estimators/bundle_adjustment.cc (~976-987)`.
    - Cost functor: `AbsolutePosePositionPriorCostFunctor (3-DoF position only)` defined in `src/colmap/estimators/cost_functions.h (~378-404)`.
    - An optional robust loss (Cauchy) can be used: `prior_options.use_robust_loss_on_prior_position` in `src/colmap/estimators/bundle_adjustment.h (~8109+)` and used in the adjuster (~906-924).

4. Gauge handling & normalization

    If priors are active, the reconstruction is normalized (scale fixed) and the priors are transformed accordingly when added; after solving, the inverse normalization is applied. See `src/colmap/estimators/bundle_adjustment.cc (~890-906, ~924-936)`.

    If there are no valid covariances, the code falls back to a standard gauge fix (fix three 3D points) and proceeds like default BA. See `src/colmap/estimators/bundle_adjustment.cc`(~896-902, and the **no valid covariance** check at ~1012-1016).

    Important: Local BA (used repeatedly during incremental registration) remains the default BA without position-prior constraints; the priors are integrated in global BA only.
    See `IncrementalMapper::IterativeLocalRefinement(...)` creating `CreateDefaultBundleAdjuster(...)` in `src/colmap/sfm/incremental_mapper.cc (~730-780)`.

    Covariances: If your database priors don't have **covariances** (NaNs), the prior constraints won't be added. **Pose prior covariance valid checking will only be performed when calling `AddPosePriorToProblem(image_t image_id, const PosePrior& prior, Reconstruction& reconstruction)` and `bool AlignReconstruction()` in `estimators/bundle_adjustment.cc`.** The CLI `pose_prior_mapper` can overwrite all covariances with a diagonal $\sigma^2$ you provide; it sets `use_prior_position=true` and runs the pipeline.
    See `src/colmap/exe/sfm.cc` (`RunPosePriorMapper`, ~356-420, ~389-396).

In short:
- Matching: Used to select candidate pairs via spatial (GPS) KNN pairing.
- SfM / BA: Used (when enabled) to align the model to the prior and to regularize camera positions in global bundle adjustment via covariance-weighted residuals with an optional robust loss.
- Not used: There's no orientation prior in the pipeline (even though generic 6-DoF prior functors exist); the implemented pipeline uses position-only priors.
### Pose prior covariance matrix
#### Build up pose prior covariance matrix derived from IMU pose + covariance matrix
COLMAP’s bundle adjustment uses position-only priors and expects a 3×3 covariance in meters, aligned with the Cartesian frame used for the prior positions at BA time:
- If coordinate_system = WGS84, COLMAP converts the positions to local ENU before BA (see `DatabaseCache::SetupPosePriors`).
Covariances are NOT transformed by COLMAP, they are taken as-is in the BA. Practically this means: store your covariance already in ENU.
- If coordinate_system = CARTESIAN, COLMAP leaves positions as-is. Then your covariance must match that Cartesian frame (use ENU if that’s what you store).

> lat $\sigma$  (meters)  -> North
>
> lon $\sigma$  (meters)  -> East
>
> alt $\sigma$  (meters)  -> Up

We can assume independence (no cross-terms) and form:
$$
\Sigma_{\text{ENU}} = diag( \sigma_E^2 , \sigma_N^2 , \sigma_U^2 )
      = diag( (lon \sigma)^2 , (lat \sigma)^2 , (alt \sigma)^2 )
$$

If your GNSS outputs NED variances, convert to ENU by swapping the first two axes and flipping the sign of Down (the sign disappears in covariance, but D→U mapping changes the axis).

It is used together with priors in BA (position residual with covariance):
- `PosePriorBundleAdjuster::AddPosePriorToProblem` in `src/colmap/estimators/bundle_adjustment.cc`
- cost: `AbsolutePosePositionPriorCostFunctor` weighted by your `position_covariance` via `CovarianceWeightedCostFunctor` in `src/colmap/estimators/cost_functions.h`

Specially we can use the GNSS-antenna position covariance matrix for the left and right cameras, if we treat the lever-arm/extrinsics and attitude as exact. In that (ideal) case the camera position is just the antenna position plus a constant offset, so the covariance is unchanged.

If an image is part of a rig, COLMAP only enforces the prior on the **reference sensor** and returns early for non-reference sensors (see comment in code `colmap/estimators/bundle_adjustment.cc` line (~1001-1008): **"Only enforce the pose prior on the reference sensor""**).
So for a stereo rig, put the prior on the reference (usually left) camera; a prior on the right camera will be ignored. 

And priors stored as WGS84 are converted to a local ENU Cartesian frame before BA, but the covariance is not transformed anywhere; it’s taken “as is” when building the BA residual. So covariance in ENU (meters) aligned with the local frame has to be provided. See code in (`src/colmap/scene/database_cache.cc` + `src/colmap/estimators/bundle_adjustment.cc`)

### Robust loss on prior position

If enabled, the BA adds a Cauchy robust loss to each position-prior residual:

```c++
prior_loss_function_ = std::make_unique<ceres::CauchyLoss>(
    prior_options_.prior_position_loss_scale);
// Source: src/colmap/estimators/bundle_adjustment.cc (class PosePriorBundleAdjuster, method AddPosePriorToProblem).
```

and every image with a valid covariance gets one residual:

```c++
problem->AddResidualBlock(
        CovarianceWeightedCostFunctor<AbsolutePosePositionPriorCostFunctor>::
            Create(prior.position_covariance,
                   normalized_from_metric_ * prior.position),
        prior_loss_function_.get(),
        cam_from_world_rotation,
        cam_from_world_translation);
// Source: src/colmap/estimators/bundle_adjustment.cc (class PosePriorBundleAdjuster, method AddPosePriorToProblem).
```
Interpretation: residuals are in whitened units (Mahalanobis distance) because the cost functor pre-multiplies by the Cholesky of your 3×3 covariance. The robust loss then down-weights cameras whose position deviates too much from their prior relative to their own covariance.

#### `normalized_from_metric_`
`normalized_from_metric_` is a 3D Sim(3) transform (scale, rotation, translation) that maps the metric reconstruction coordinates (after aligning the model to your priors) into the normalized coordinates used while solving the BA. It is defined in `colmap/src/colmap/scene/reconstruction.h` like:
```c++
  // Normalize scene by scaling and translation to improve numerical stability
  // of algorithms.
  //
  // Translates scene such that the mean of the camera centers or point
  // locations are at the origin of the coordinate system.
  //
  // Scales scene such that the minimum and maximum camera centers (or points)
  // are at the given `extent`, whereas `min_percentile` and `max_percentile`
  // determine the minimum and maximum percentiles of the camera centers (or
  // points) considered.
  Sim3d Normalize(bool fixed_scale = false,
                  double extent = 10.0,
                  double min_percentile = 0.1,
                  double max_percentile = 0.9,
                  bool use_images = true);
```

##### How it's computed (step-by-step):

1. Align to priors (metric scale):

Before any BA, COLMAP robustly aligns the current reconstruction to your prior positions by estimating a Sim(3) with RANSAC:
```c++
Sim3d metric_from_orig;
const bool success = AlignReconstructionToPosePriors(
    reconstruction_, pose_priors_, ransac_options, &metric_from_orig);
if (success) {
  reconstruction_.Transform(metric_from_orig);
}
// file: estimators/bundle_adjustment.cc (AlignReconstruction()).
```
2. Normalize (recenter only):
If priors are usable, they then normalize the reconstruction without changing the scale (just recenters to improve conditioning):
```c++
// Normalize the reconstruction to avoid numerical instability but
// do not transform priors as they will be transformed when added.
normalized_from_metric_ = reconstruction_.Normalize(/*fixed_scale=*/true);
// file: estimators/bundle_adjustment.cc
```
The implementation of `Normalize(fixed_scale=true)` translates the model so its centroid is near the origin and keeps `scale = 1` (no scaling when `fixed_scale` is true):

```c++
Sim3d Reconstruction::Normalize(const bool fixed_scale,
                                const double extent,
                                const double min_percentile,
                                const double max_percentile,
                                const bool use_images) {
  THROW_CHECK_GT(extent, 0);

  if ((use_images && NumRegFrames() < 2) ||
      (!use_images && points3D_.size() < 2)) {
    return Sim3d();
  }

  const auto [bbox, centroid] =
      ComputeBBBoxAndCentroid(min_percentile, max_percentile, use_images);

  // Calculate scale and translation, such that
  // translation is applied before scaling.
  double scale = 1.;
  if (!fixed_scale) {
    const double old_extent = bbox.diagonal().norm();
    if (old_extent >= std::numeric_limits<double>::epsilon()) {
      scale = extent / old_extent;
    }
  }

  Sim3d tform(scale, Eigen::Quaterniond::Identity(), -scale * centroid);
  Transform(tform);

  return tform;
}
// file: scene/reconstruction.cc and declaration in scene/reconstruction.h
```
3. Use it to keep frames consistent:
When adding each prior residual to Ceres, they transform the stored prior position into the same normalized frame:
```c++
problem->AddResidualBlock(
  CovarianceWeightedCostFunctor<AbsolutePosePositionPriorCostFunctor>::Create(
    prior.position_covariance,
    normalized_from_metric_ * prior.position),
  prior_loss_function_.get(),
  cam_from_world_rotation, cam_from_world_translation);
// file: estimators/bundle_adjustment.cc (AddPosePriorToProblem).
```

4. Undo normalization after solving:
After BA, they bring the reconstruction back to the metric frame:
```c++
reconstruction_.Transform(Inverse(normalized_from_metric_));
// same file, Solve().
```
##### Why it's needed:
- Numerical conditioning. Bringing camera centers near the origin (and optionally to a standard scale) makes Jacobians better conditioned and solvers happier.
- Frame consistency. Because the BA runs in the normalized frame, the priors have to be evaluated in that same frame. Saving normalized_from_metric_ lets them transform every prior consistently when constructing residuals.
- No covariance mismatch. With `fixed_scale=true`, normalization is pure translation (no scale, no rotation). Translation does not change a covariance, so they can safely reuse your 3×3 covariance as-is.

### Prior position loss scale
This is the Cauchy scale a used above. The header even gives the intended scale:

```c++
// (chi2 for 3DOF at 95% = 7.815).
double prior_position_loss_scale = 7.815;
// Source: src/colmap/estimators/bundle_adjustment.h (PosePriorBundleAdjustmentOptions).
```
Because the residual is the squared Mahalanobis norm, 7.815 is the χ²(3 dof, 95%) cutoff. In practice:

If an image's prior error ≲ √7.815 ≈ 2.8 σ, it behaves ~quadratic (full weight).

Far beyond that, the Cauchy loss suppresses that prior, so a few bad GNSS fixes don't drag the whole model.

#### Where `prior_position_loss_scale = 7.815` comes from, and how it works

##### Where defined:
```c++
struct PosePriorBundleAdjustmentOptions {
  bool use_robust_loss_on_prior_position = false;
  // Threshold on the residual for the robust loss
  // (chi2 for 3DOF at 95% = 7.815).
  double prior_position_loss_scale = 7.815;
  double ransac_max_error = 0.;
};
// src/colmap/estimators/bundle_adjustment.h
```
##### Where used:
If you enable the robust loss on priors, they instantiate a Ceres Cauchy loss with that scale:
```c++
if (prior_options_.use_robust_loss_on_prior_position) {
  prior_loss_function_ = std::make_unique<ceres::CauchyLoss>(
      prior_options_.prior_position_loss_scale);
}
// estimators/bundle_adjustment.cc (constructor of PosePriorBundleAdjuster).
```

##### What residual `s` the loss sees:
The prior residual is the Mahalanobis squared error of the camera center vs. the prior position:
- The 3-vector residual is 
    $$r = p_{prior} - (-R^{T}t)$$
    implemented as `position_in_world_prior + q^{-1} * t` in
    `AbsolutePosePositionPriorCostFunctor`.

- It is whitened by your pose prior covariance $\Sigma$ via a left square-root information:
    $$ \text{LeftSqrtInformation}(\Sigma) = (\Sigma^{-1})^{1/2}$$
    and implementation:
    ```c++
    CovMat LeftSqrtInformation(const CovMat& cov) {
        return cov.inverse().llt().matrixL().transpose();
    }
    // file: estimators/cost_functions.h.
    ```
    using CovarianceWeightedCostFunctor, so the scalar Ceres residual is:
    $$s = r^{T}\Sigma^{-1}r$$
    

##### Why `7.815`?
For 3 independent Gaussian components (E,N,U), the statistic 
$s = r^{T}\Sigma^{-1}r$ follows a chi-square distribution with 3 dof (degree of freedom). The 95% quantile is 7.815. That means:

- If the error is within roughly $\sqrt{7.815} \approx 2.80 \sigma$, it lies inside a 95% confidence ellipsoid, treat it as **inlier-ish**.
- Beyond that, we'd like to **down-weight** the influence of the prior to avoid one bad GPS fix dominating the solve.

##### How Cauchy uses the scale:
Ceres' Cauchy loss with scale $a$ applies to the squared residual $s$ as
$$\rho(s) = a^2 \log{(1 + \frac{s}{a^2})}$$
$$\omega(s) = \frac{1}{1+ s/ a^2}$$
where $\omega$ is the effective weight factor applied in the solver. Setting $a = 7.815$ places the soft threshold right at the 95% chi-square boundary for 3D. Inside that region, the behavior is close to quadratic (L2); outside, weights decay roughly like $1/s$.

Practical takeaway:
- Turn it on if your priors can be occasionally wrong (GNSS multipath, outages, mixed sources). The default 7.815 is sensible.
- If you have high-grade RTK/INS with accurate covariances, you can leave robust loss off for slightly tighter pulls from the priors, or keep it on for safety.

##### Quick proof it's doing what we said
- Look for these lines in the log when running the prior-aware mapper:
    - "Robustly aligning reconstruction with max_error=…" (from `AlignReconstruction()`).
    - After success, VLOG(2) prints RMSE/median error to priors.
- In code where each prior residual is added you will see both pieces we discussed, the normalized transform and the covariance weighting + optional Cauchy:
```c++
problem->AddResidualBlock(
  CovarianceWeightedCostFunctor<AbsolutePosePositionPriorCostFunctor>::Create(
    prior.position_covariance,
    normalized_from_metric_ * prior.position),
  prior_loss_function_.get(),
  cam_from_world_rotation, cam_from_world_translation);
// file: estimators/bundle_adjustment.cc (AddPosePriorToProblem).
```
### Mismatch between the GUI and the option system
#### What the GUI exposes
In `src/colmap/ui/reconstruction_options_widget.cc` the Priors tab is created with:

```c++
class MapperPriorsOptionsWidget : public OptionsWidget {
 public:
  MapperPriorsOptionsWidget(QWidget* parent, OptionManager* options)
      : OptionsWidget(parent) {
    AddOptionBool(&options->mapper->use_prior_position, "use_prior_position");
    AddOptionBool(&options->mapper->use_robust_loss_on_prior_position,
                  "use_robust_loss_on_prior_position");
    AddOptionDouble(&options->mapper->prior_position_loss_scale,
                    "prior_position_loss_scale");
  }
};
```
So when you tick those boxes in the GUI, the values in `options->mapper->{use_prior_position, use_robust_loss_on_prior_position, prior_position_loss_scale}` do change in memory. The base class `OptionsWidget` writes the values back on close/hide (`closeEvent/hideEvent` call `WriteOptions()`), so they're in effect for the **current run**.

And they are consumed at runtime:
- `IncrementalPipeline::LoadDatabase()` checks `options_->use_prior_position` and calls `DatabaseCache::SetupPosePriors()` if `true`.
- Global BA uses the robust loss and loss scale when `use_prior_position` is true (see `PosePriorBundleAdjuster` in `estimators/bundle_adjustment.cc`).

You can verify it's active by looking for the log **`Setting up prior positions...`**

#### Why it's not saved to the project `.ini` (and not in `--help`)

The `OptionManager` only writes options that have been registered with it. In `src/colmap/controllers/option_manager.cc`, the function `OptionManager::AddMapperOptions()` registers a long list of `Mapper.* keys`, but it does not register the three prior options. You can see the block ends with triangulation options like:
```c++
AddAndRegisterDefaultOption("Mapper.tri_ignore_two_view_tracks",
                            &mapper->triangulation.ignore_two_view_tracks);
```

and there is no:
```c++
AddAndRegisterDefaultOption("Mapper.use_prior_position",
                            &mapper->use_prior_position);
AddAndRegisterDefaultOption("Mapper.use_robust_loss_on_prior_position",
                            &mapper->use_robust_loss_on_prior_position);
AddAndRegisterDefaultOption("Mapper.prior_position_loss_scale",
                            &mapper->prior_position_loss_scale);
```

Two consequences
- Saving the project: MainWindow call `options_.Write(*options_.project_path)` to save registered keys. Since three keys are not registered, they are not written to the `.ini`.

- Command-line `--help` for colmap mapper doesn't show these three options since they are not registered.