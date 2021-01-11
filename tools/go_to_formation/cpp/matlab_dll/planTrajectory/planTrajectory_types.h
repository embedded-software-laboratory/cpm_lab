//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_types.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef PLANTRAJECTORY_TYPES_H
#define PLANTRAJECTORY_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#ifdef PLANTRAJECTORY_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define PLANTRAJECTORY_DLL_EXPORT      __declspec(dllimport)
#else
#define PLANTRAJECTORY_DLL_EXPORT
#endif

#elif defined(BUILDING_PLANTRAJECTORY)
#if defined(_MSC_VER) || defined(__LCC__)
#define PLANTRAJECTORY_DLL_EXPORT      __declspec(dllexport)
#else
#define PLANTRAJECTORY_DLL_EXPORT      __attribute__ ((visibility("default")))
#endif

#else
#define PLANTRAJECTORY_DLL_EXPORT
#endif

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4251)

#endif

// Type Definitions
namespace mgen
{
  struct cell_wrap_38
  {
    char f1[3];
  };

  class PLANTRAJECTORY_DLL_EXPORT DubinsBuiltins
  {
   public:
    static void autonomousDubinsDistance(const coder::array<double, 2U>
      &startPose, const double goalPose_data[], const int goalPose_size[2],
      double turningRadius, coder::array<double, 1U> &dist);
    static void autonomousDubinsInterpolate(const double startPose[3], const
      double goalPose_data[], double connectionDistance, double numSteps, double
      turningRadius, coder::array<double, 2U> &poses);
    static void autonomousDubinsDistance(const coder::array<double, 2U>
      &startPose, const double goalPose[3], double turningRadius, coder::array<
      double, 1U> &dist);
    static double autonomousDubinsDistance(const double startPose[3], const
      double goalPose[3], double turningRadius);
    static void b_autonomousDubinsInterpolate(const double startPose[3], const
      double goalPose[3], double connectionDistance, double numSteps, double
      turningRadius, coder::array<double, 2U> &poses);
    static void autonomousDubinsDistance(const double startPose[3], const coder::
      array<double, 2U> &goalPose, double turningRadius, coder::array<double, 1U>
      &dist);
    static void autonomousDubinsSegments(const double startPose[3], const double
      goalPose[3], double turningRadius, const coder::array<cell_wrap_38, 2U>
      &disabledTypes, double *cost, double motionLengths[3], double motionTypes
      [3]);
    static void c_autonomousDubinsInterpolateSe(const double startPose[3], const
      double goalPose[3], const coder::array<double, 2U> &samples, double
      turningRadius, const double segmentsLengths[3], const unsigned int
      segmentsTypes[3], coder::array<double, 2U> &poses);
    DubinsBuiltins();
    ~DubinsBuiltins();
  };

  class PLANTRAJECTORY_DLL_EXPORT Pose2D
  {
   public:
    void init(double b_x, double b_y, double b_yaw);
    Pose2D();
    ~Pose2D();
    double x;
    double y;
    double yaw;
  };

  class PLANTRAJECTORY_DLL_EXPORT angleUtilities
  {
   public:
    static double angdiff(double x, double y);
    static void convertAndWrapTo2Pi(double *theta);
    static void wrapTo2Pi(double *theta);
    static void wrapToPi(double *theta);
    angleUtilities();
    ~angleUtilities();
  };

  class PLANTRAJECTORY_DLL_EXPORT c_driving_internal_costmap_Vehi
  {
   public:
    c_driving_internal_costmap_Vehi *init();
    void setCosts(const double xyPoints[8]);
    void VehicleCostmapImpl_setCosts(const double xyPoints[2]);
    void setCosts();
    c_driving_internal_costmap_Vehi *copy(c_driving_internal_costmap_Vehi
      *iobj_0) const;
    double get_FreeThreshold() const;
    double get_OccupiedThreshold() const;
    c_driving_internal_costmap_Vehi *b_init(const double varargin_1[180000],
      const double varargin_5[2], double varargin_9, double varargin_11, const
      boolean_T varargin_13[180000], const boolean_T varargin_15[180000]);
    void get_MapExtent(double mapExtent[4]) const;
    void checkFreePoses(const double vehiclePoses[15000], boolean_T b_free[5000])
      const;
    void checkFreeWorldPoints(const double xyPoints[30000], boolean_T b_free
      [15000]) const;
    void b_checkFreePoses(const double vehiclePoses[6], boolean_T b_free[2])
      const;
    void b_checkFreeWorldPoints(const double xyPoints[12], boolean_T b_free[6])
      const;
    void c_checkFreePoses(const coder::array<double, 2U> &vehiclePoses, coder::
                          array<boolean_T, 1U> &b_free) const;
    void c_checkFreeWorldPoints(const coder::array<double, 2U> &xyPoints, coder::
      array<boolean_T, 1U> &b_free) const;
    c_driving_internal_costmap_Vehi();
    ~c_driving_internal_costmap_Vehi();
   protected:
    void inflate();
    void xyPointsToGridIndices(const double xyPoints[3400], double gridIndices
      [1700]) const;
    void vehiclePoseToWorldPoints(const double vehiclePoses[15000], double
      xyPoints[30000]) const;
    void xyPointsToGridIndices(const coder::array<double, 2U> &xyPoints, coder::
      array<double, 1U> &gridIndices) const;
   public:
    double Costmap[180000];
    boolean_T OccupiedMap[180000];
    boolean_T FreeMap[180000];
   protected:
    double MapLocation[2];
    double CollisionCheckOffsets[3];
    double pFreeThreshold;
    double pOccupiedThreshold;
  };

  class PLANTRAJECTORY_DLL_EXPORT e_driving_internal_planning_Dub
  {
   public:
    static e_driving_internal_planning_Dub *create
      (e_driving_internal_planning_Dub *iobj_0);
    void set_MinTurningRadius(double radius);
    void set_MinTurningRadius();
    void connectInternal(const double startPose[3], const double goalPose[3],
                         double motionLengths[3], char motionTypes[3]) const;
    e_driving_internal_planning_Dub();
    ~e_driving_internal_planning_Dub();
    double MinTurningRadius;
    coder::array<cell_wrap_38, 2U> DisabledPathTypesInternal;
  };

  class PLANTRAJECTORY_DLL_EXPORT c_driving_internal_planning_Dub
  {
   public:
    static void create(const e_driving_internal_planning_Dub *varargin_1,
                       c_driving_internal_planning_Dub *b_this);
    void init(const e_driving_internal_planning_Dub *varargin_1);
    void get_StartPose(double startPose[3]) const;
    static void create(const e_driving_internal_planning_Dub *varargin_1, const
                       double varargin_2[3], const double varargin_3[3],
                       c_driving_internal_planning_Dub *b_this);
    void init(const e_driving_internal_planning_Dub *varargin_1, const double
              varargin_2[3], const double varargin_3[3]);
    double get_Length() const;
    void interpolateInternal(const coder::array<double, 1U> &varargin_1,
      boolean_T varargin_2, coder::array<double, 2U> &poses, coder::array<double,
      1U> &directions) const;
    c_driving_internal_planning_Dub();
    ~c_driving_internal_planning_Dub();
    double MinTurningRadius;
    double MotionLengths[3];
    char MotionTypes[3];
   protected:
    double StartPoseInternal[3];
    double GoalPoseInternal[3];
  };

  struct cell_41
  {
    double f1;
  };

  class PLANTRAJECTORY_DLL_EXPORT c_matlabshared_autonomous_core_
  {
   public:
    c_matlabshared_autonomous_core_ *init();
    void parse();
    double parameterValue() const;
    c_matlabshared_autonomous_core_();
    ~c_matlabshared_autonomous_core_();
    cell_41 Defaults;
   private:
    cell_41 ParsedResults;
  };

  class PLANTRAJECTORY_DLL_EXPORT c_matlabshared_planning_interna
  {
   public:
    c_matlabshared_planning_interna *init();
    void distance(const coder::array<double, 2U> &from, const double to_data[],
                  const int to_size[2], coder::array<double, 1U> &d) const;
    void interpolate(const double from[3], const double to_data[], coder::array<
                     double, 2U> &poses) const;
    void distance(const coder::array<double, 2U> &from, const double to[3],
                  coder::array<double, 1U> &d) const;
    double distance(const double from[3], const double to[3]) const;
    void b_interpolate(const double from[3], const double to[3], coder::array<
                       double, 2U> &poses) const;
    void distance(const double from[3], const coder::array<double, 2U> &to,
                  coder::array<double, 1U> &d) const;
    c_matlabshared_planning_interna();
    ~c_matlabshared_planning_interna();
    double ConnectionDistance;
    double NumSteps;
    double TurningRadius;
  };

  class PLANTRAJECTORY_DLL_EXPORT coder_internal_stack
  {
   public:
    void init();
    coder_internal_stack();
    ~coder_internal_stack();
    coder::array<double, 1U> d;
    int n;
  };

  class PLANTRAJECTORY_DLL_EXPORT d_driving_internal_planning_Dub
  {
   public:
    static void makeempty(d_driving_internal_planning_Dub *e);
    static void create(const e_driving_internal_planning_Dub *varargin_1,
                       d_driving_internal_planning_Dub *b_this);
    static void create(const e_driving_internal_planning_Dub *varargin_1, const
                       double varargin_2[3], const double varargin_3[3],
                       d_driving_internal_planning_Dub *b_this);
    double numel() const;
    boolean_T isrow() const;
    boolean_T isempty() const;
    void get_Length(coder::array<double, 1U> &len) const;
    void get_StartPose(double startPose_data[], int startPose_size[2]) const;
    void get_MotionLengths(double motionLens_data[], int motionLens_size[2])
      const;
    void get_Length(double len_data[], int len_size[1]) const;
    void interpolateInternal(const coder::array<double, 1U> &varargin_1,
      boolean_T varargin_2, coder::array<double, 2U> &varargout_1, coder::array<
      double, 1U> &varargout_2) const;
    double length() const;
    void repmat();
    void parenAssign(const d_driving_internal_planning_Dub *rhs, double idx);
    void parenReference();
    void parenReference(double idx);
    d_driving_internal_planning_Dub();
    ~d_driving_internal_planning_Dub();
    coder::array<c_driving_internal_planning_Dub, 2U> Data;
  };

  struct cell_wrap_8
  {
    char f1[7];
  };

  class PLANTRAJECTORY_DLL_EXPORT d_matlabshared_autonomous_core_
  {
   public:
    d_matlabshared_autonomous_core_ *init();
    void parse();
    void parameterValue(char value[7]) const;
    d_matlabshared_autonomous_core_();
    ~d_matlabshared_autonomous_core_();
    cell_wrap_8 Defaults[1];
   private:
    cell_wrap_8 ParsedResults[1];
  };

  class PLANTRAJECTORY_DLL_EXPORT d_matlabshared_planning_interna
  {
   public:
    d_matlabshared_planning_interna *init(c_driving_internal_costmap_Vehi
      *costmap);
    void reset();
    void configureCollisionChecker();
    double sampleGoalBias();
    void sampleCollisionFree(double pose_data[], int pose_size[2]);
    void sample(double pose_data[], int pose_size[2], boolean_T *collisionFree);
    d_matlabshared_planning_interna();
    ~d_matlabshared_planning_interna();
   private:
    void fillPoseBuffer();
    void attemptResampling();
    double PoseBuffer[15000];
    double PoseIndex;
    double GoalBiasBuffer[5000];
    double GoalBiasIndex;
    double LowerLimits[3];
    double UpperLimits[3];
    c_driving_internal_costmap_Vehi *Costmap;
    boolean_T CollisionFree[5000];
  };

  class PLANTRAJECTORY_DLL_EXPORT driving_Path
  {
   public:
    static void create(const d_driving_internal_planning_Dub *varargin_1,
                       driving_Path *b_this);
    void interpolate(const coder::array<double, 2U> &varargin_1, coder::array<
                     double, 2U> &poses) const;
    void interpolate(const coder::array<double, 2U> &varargin_1, coder::array<
                     double, 2U> &poses, coder::array<double, 1U> &directions)
      const;
    void get_StartPose(double startPose_data[], int startPose_size[2]) const;
    double get_Length() const;
    void interpolate(coder::array<double, 2U> &poses) const;
    void interpolate(coder::array<double, 2U> &poses, coder::array<double, 1U>
                     &directions) const;
    driving_Path();
    ~driving_Path();
    d_driving_internal_planning_Dub PathSegments;
  };

  class PLANTRAJECTORY_DLL_EXPORT e_matlabshared_planning_interna
  {
   public:
    e_matlabshared_planning_interna *init(c_matlabshared_planning_interna
      *varargin_1);
    void reset();
    void b_nearest(const double nodeBuffer[30003], double numNodes, const double
                   node_data[], const int node_size[2], double nearestNode[3],
                   double *nearestId);
    void distance(const coder::array<double, 2U> &from, const double to_data[],
                  const int to_size[2], coder::array<double, 1U> &d) const;
    void near(const double nodeBuffer[30003], double numNodes, const double
              node[3], double K, coder::array<double, 2U> &nearNodes, coder::
              array<double, 1U> &nearIds);
    void distance(const coder::array<double, 2U> &from, const double to[3],
                  coder::array<double, 1U> &d) const;
    double distance(const double from[3], const double to[3]) const;
    e_matlabshared_planning_interna();
    ~e_matlabshared_planning_interna();
   private:
    void computeOffsets(const double *numNodes, double *offset, double *step);
   public:
    c_matlabshared_planning_interna *ConnectionMechanism;
   private:
    double Offset;
  };

  class PLANTRAJECTORY_DLL_EXPORT f_matlabshared_planning_interna
  {
   public:
    f_matlabshared_planning_interna *init(e_matlabshared_planning_interna
      *neighborSearcher);
    void reset();
    void addNode(const double node[3]);
    void b_nearest(const double node_data[], const int node_size[2], double
                   nearestNode[3], double *nearestId);
    void near(const double node[3], coder::array<double, 2U> &nearNodes, coder::
              array<double, 1U> &nearIds);
    double b_addNode(const double node[3]);
    double costTo(double id) const;
    void addEdge(double fromId, double toId);
    double edgeCost(double fromId, double toId) const;
    void replaceParent(double childId, double newParentId);
    void get_Edges(coder::array<unsigned int, 2U> &edges) const;
    double costTo(unsigned int id) const;
    double edgeCost(unsigned int fromId, unsigned int toId) const;
    void shortestPathFromRoot(const double childId_data[], const int
      childId_size[2], const coder::array<double, 2U> &goalNodeIds, unsigned int
      path_data[], int path_size[2], double totalCost_data[], int
      totalCost_size[2]) const;
    void costTo(const double id_data[], const int id_size[2], double cost_data[],
                int cost_size[2]) const;
    void get_Nodes(coder::array<double, 2U> &nodes) const;
    f_matlabshared_planning_interna();
    ~f_matlabshared_planning_interna();
   private:
    void c_rectifyDownstreamCostsNoRecur(double childId);
   public:
    e_matlabshared_planning_interna *NeighborSearcher;
    double NodeBuffer[30003];
    double NodeIndex;
    unsigned int EdgeBuffer[20002];
    double EdgeIndex;
    double CostBuffer[10001];
  };

  class PLANTRAJECTORY_DLL_EXPORT g_matlabshared_planning_interna
  {
   public:
    g_matlabshared_planning_interna *init(c_driving_internal_costmap_Vehi
      *costmap, const double goalTolerance[3], e_matlabshared_planning_interna
      *iobj_0, c_matlabshared_planning_interna *iobj_1);
    void planPath(const double startPose[3], const double goalPose[3], coder::
                  array<double, 2U> &varargout_1);
    g_matlabshared_planning_interna();
    ~g_matlabshared_planning_interna();
   protected:
    void findMinCostPath(const coder::array<double, 2U> &nearPoses, const coder::
                         array<double, 1U> &nearIds, const double nearestPose[3],
                         double nearestId, const double newPose[3], double newId);
    void rewireTree(const coder::array<double, 2U> &nearPoses, const coder::
                    array<double, 1U> &nearIds, const double newPose[3], double
                    newId);
    boolean_T inGoalRegion(const double pose[3]) const;
   public:
    c_driving_internal_costmap_Vehi *Costmap;
    c_matlabshared_planning_interna *ConnectionMechanism;
    d_matlabshared_planning_interna Sampler;
    f_matlabshared_planning_interna Tree;
    double StartPose[3];
    double GoalPose[3];
    double GoalTolerance[3];
    double GoalBias;
    double MinIterations;
    double MaxIterations;
  };

  struct struct0_T
  {
    unsigned char vehicle_id;
    Pose2D pose;
  };

  class PLANTRAJECTORY_DLL_EXPORT pathPlannerRRT
  {
   public:
    pathPlannerRRT *init(c_driving_internal_costmap_Vehi *varargin_1,
                         e_matlabshared_planning_interna *iobj_0,
                         c_matlabshared_planning_interna *iobj_1,
                         c_driving_internal_costmap_Vehi *iobj_2);
    void plan(double startPose[3], double goalPose[3], driving_Path *varargout_1);
    double get_MinTurningRadius() const;
    pathPlannerRRT();
    ~pathPlannerRRT();
   private:
    void validatePoses(const double startPose[3], const double goalPose[3])
      const;
    void createPath(const coder::array<double, 2U> &pathPoses, driving_Path
                    *refPath) const;
   public:
    c_driving_internal_costmap_Vehi *Costmap;
    g_matlabshared_planning_interna InternalPlanner;
   private:
    driving_Path Path;
  };

  struct struct1_T
  {
    unsigned long t;
    double px;
    double py;
    double vx;
    double vy;
  };
}

#define MAX_THREADS                    omp_get_max_threads()
#ifdef _MSC_VER

#pragma warning(pop)

#endif
#endif

//
// File trailer for planTrajectory_types.h
//
// [EOF]
//
