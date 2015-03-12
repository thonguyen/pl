#ifndef LB_STEPDETECTDEFECTSPCL_H
#define LB_STEPDETECTDEFECTSPCL_H

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_tools/model/ct_autorenamemodels.h"
#include "ct_itemdrawable/ct_scene.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>

/*!
 * \class LB_StepDetectDefectsPcl
 * \ingroup Steps_LB
 * \brief <b>Detect external defects.</b>
 *
 * Detect external defects using PCL
 *
 *
 */

class LB_StepDetectDefectsPcl: public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     * 
     * Create a new instance of the step
     * 
     * \param dataInit Step parameters object
     */
    LB_StepDetectDefectsPcl(CT_StepInitializeData &dataInit);

    /*! \brief Step description
     * 
     * Return a description of the step function
     */
    QString getStepDescription() const;

    /*! \brief Step detailled description
     * 
     * Return a detailled description of the step function
     */
    QString getStepDetailledDescription() const;

    /*! \brief Step URL
     * 
     * Return a URL of a wiki for this step
     */
    QString getStepURL() const;

    /*! \brief Step copy
     * 
     * Step copy, used when a step is added by step contextual menu
     */
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);

protected:

    /*! \brief Input results specification
     * 
     * Specification of input results models needed by the step (IN)
     */
    void createInResultModelListProtected();

    /*! \brief Parameters DialogBox
     * 
     * DialogBox asking for step parameters
     */
    void createPostConfigurationDialog();

    /*! \brief Output results specification
     * 
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Algorithm of the step
     * 
     * Step computation, using input results, and creating output results
     */
    void compute();

    /*! \brief Estimate a cylinder from a point cloud of billon
     * Return a model coefficents of cylinder
     */
    pcl::ModelCoefficients::Ptr estimateCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices &inliersCylinder);

    /*! \brief Segment the log by angle
     *
     */
    void segmentByAngle(pcl::PointCloud<pcl::PointXYZ> cloud,
                        std::vector<pcl::PointIndices> sliceIndices,
                        double angleStep,
                        std::vector<pcl::PointIndices::Ptr> &cloudPartIndices);


    double distanceToPlane(Eigen::Vector3d point, Eigen::Vector3d normal, double d);

    /*! \brief Detect points whose distance to plane is greater than a threshold
     *  A point with its distance to plane > threshold is considered like a defect point
     */
    std::vector<pcl::ModelCoefficients::Ptr>
    detectDefaults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   std::vector<pcl::PointIndices::Ptr> cloudPartIndices,
                   double threshold,
                   pcl::PointIndices &defautls);
    /*! \brief A transformation matrix which does a translation the barry center of the log to the origin
     *  and two rotation about Oy and Oz to align the axis of the log with Ox axis
     *
     */
    Eigen::Matrix4d getTransformationMatrix(Eigen::Vector3d directionVector, Eigen::Vector3d translationVector);

    /*! \brief Transform a point cloud by a transformation matrix
     *  The input point cloud is also transformed
     */
    void transformCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Matrix4d transMat, CT_Scene *outScene);

    void transformCloudByEnertia(pcl::PointCloud<pcl::PointXYZ> &cloud, CT_Scene *outScene);
    /*! \brief Transform a point cloud by a transformation matrix
     *
     */
    void sceneToPCLCloud(CT_Scene *scene, pcl::PointCloud<pcl::PointXYZ> &cloud);

    /*! \brief Clustering the defect point cloud
     *
     */
    void clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           pcl::PointIndices::Ptr defaultIndices,
                           std::vector<pcl::PointIndices> &clusterIndices);

    /*! \brief Cut the log into small slices
     *
     */
    void cutLog(pcl::PointCloud<pcl::PointXYZ> cloud,
                double sliceLength,
                std::vector<pcl::PointIndices> &sliceIndices);

    /*! \brief preprocess the point cloud
     *  remove outliers, the plain at termius of the log
     */
    void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud);

    /*! \brief preprocess the point cloud
     *  remove outliers, the plain at termius of the log
     */
    Eigen::Matrix3d getEigenVectors(pcl::PointCloud<pcl::PointXYZ> cloud);

    Eigen::Vector3d getCentroid(pcl::PointCloud<pcl::PointXYZ> cloud);

private:
    double maxX;
    double maxY;

    double                  distanceThreshold;
    double                  angleThreshold;
    CT_AutoRenameModels     outCylinderModelName;
    CT_AutoRenameModels     outClusterModelName;
    CT_AutoRenameModels     outGroupClusterModelName;
    CT_AutoRenameModels     outGroupModelName;
    CT_AutoRenameModels     outSceneModelName;
};

#endif // LB_STEPDETECTDEFECTSPCL_H
