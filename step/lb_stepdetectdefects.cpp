#include "lb_stepdetectdefects.h"

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

#include "ct_itemdrawable/ct_cylinder.h"

#include "ct_result/tools/iterator/ct_resultgroupiterator.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_result/ct_resultgroup.h"

#include "pcl/common/common.h"
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#define DEF_IN_RESULT       "InResult"
#define DEF_IN_GROUP        "InGroup"
#define DEF_IN_POINT_CLOUD  "InPoints"
#define DEF_IN_SCENE        "InScene"
#define DEF_OUT_RESULT      "OutResult"
#define DEF_OUT_GROUP       "OutGroup"
#define DEF_OUT_CLUSTER     "OutCluster"
#define DEF_OUT_CIRCLE      "OutCircle"

// Constructor : initialization of parameters
LB_StepDetectDefects::LB_StepDetectDefects(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

// Step description (tooltip of contextual menu)
QString LB_StepDetectDefects::getStepDescription() const
{
    return tr("Detect log defects");
}

// Step detailled description
QString LB_StepDetectDefects::getStepDetailledDescription() const
{
    return tr("Detect log defects");
}

// Step URL
QString LB_StepDetectDefects::getStepURL() const
{
    //return tr("STEP URL HERE");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* LB_StepDetectDefects::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new LB_StepDetectDefects(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void LB_StepDetectDefects::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy* resultInModel =  createNewInResultModelForCopy(DEF_IN_RESULT, tr("Scene(s)"));
    resultInModel->setZeroOrMoreRootGroup();
    resultInModel->addGroupModel("", DEF_IN_GROUP, CT_AbstractItemGroup::staticGetType(), tr("Cluster(s)"));
    resultInModel->addItemModel(DEF_IN_GROUP,
                                DEF_IN_POINT_CLOUD,
                                CT_AbstractItemDrawableWithPointCloud::staticGetType(),
                                tr("Scène"));
}

// Creation and affiliation of OUT models
void LB_StepDetectDefects::createOutResultModelListProtected()
{
    // No OUT model definition => create an empty result
    CT_OutResultModelGroupToCopyPossibilities *resultOut = createNewOutResultModelToCopy(DEF_IN_RESULT);
    resultOut->addItemModel(DEF_IN_GROUP, _outCylinderModelName, new CT_Cylinder(), tr("Cylindre"));
    resultOut->addItemModel(DEF_IN_GROUP, _outClusterModelName, new CT_PointCluster(), tr("Cluster"));
    //resultOut->addItemModel(DEFin_grpCluster, _outCircleModelName, new CT_Circle(), tr("Cycle"));
}

// Semi-automatic creation of step parameters DialogBox
void LB_StepDetectDefects::createPostConfigurationDialog()
{
    // No parameter dialog for this step
}

void LB_StepDetectDefects::compute()
{
    CT_ResultGroup* outResult = (CT_ResultGroup*) getOutResultList().first();
    CT_ResultGroupIterator it(outResult, this, DEF_IN_GROUP);
    pcl::PointCloud<CT_Point>::Ptr cloud(new pcl::PointCloud<CT_Point>);

    while(it.hasNext() && !isStopped()){
        CT_AbstractItemGroup* group = (CT_AbstractItemGroup*)it.next();

        const CT_AbstractItemDrawableWithPointCloud* item =
                (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, DEF_IN_POINT_CLOUD);
        if(item != NULL)
        {
            // on récupère le nuage d'indices de la scène
            CT_PCIR pcir = item->getPointCloudIndexRegistered();
            CT_PointCluster * pointCluster = new CT_PointCluster(_outClusterModelName.completeName(), outResult);

            // création de l'itérateur
            CT_PointIterator itPoint(pcir);
            bool firstPosition = true;
            while(itPoint.hasNext() && !isStopped()){
                itPoint.next();
                // récupération du point
                const CT_Point &p = itPoint.currentPoint();
                pointCluster->addPoint(itPoint.cIndex(),false, firstPosition);
                cloud->points.push_back(p);
                firstPosition = false;
            }

            //get the point cloud from item
            //loop through all points to remove outliers
            CT_CylinderData *cylinderData = CT_CylinderData::staticCreate3DCylinderDataFromPointCloud(*pointCluster->getPointCloudIndex(),
                                                                                                      Eigen::Vector3d(pointCluster->getBarycenter().x(),
                                                                                                                      pointCluster->getBarycenter().y(),
                                                                                                                      pointCluster->getBarycenter().z()));

            //get barrycenter
            Eigen::Vector3d barryCenter(pointCluster->getBarycenter().x(), pointCluster->getBarycenter().y(), pointCluster->getBarycenter().z());

            double radius = 30;

            //get all points within the sphere whose center is barryCenter and radius is radius
            QVector<CT_Point> sample;
            CT_PointIterator iteratorPoint(pointCluster->getPointCloudIndexRegistered());
            while(iteratorPoint.hasNext() && !isStopped()){
                iteratorPoint.next();
                const CT_Point &p = itPoint.currentPoint();
                //distance from p to barry center
                double distanceToCenter = sqrt((p.x() - barryCenter.x()) * (p.x() - barryCenter.x()) +
                                               (p.y() - barryCenter.y()) * (p.y() - barryCenter.y()) +
                                               (p.z() - barryCenter.z()) * (p.z() - barryCenter.z()));
                if(distanceToCenter < radius)
                {
                    sample.push_back(p);
                }
            }

            //estimate axis direction of sample
            pcl::KdTreeFLANN<CT_Point> kdtree;
            kdtree.setInputCloud(cloud);
            int K = 10;
            for (int i = 0; i < sample.size(); ++i) {
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                kdtree.nearestKSearch (sample.at(i), K, pointIdxNKNSearch, pointNKNSquaredDistance);
            }




            if(cylinderData != NULL && cylinderData->getRadius() > 10)
            {
                CT_Cylinder *cyl = new CT_Cylinder(_outCylinderModelName.completeName(), outResult, cylinderData);
                group->addItemDrawable(cyl);
                PS_LOG->addMessage(LogInterface::debug, LogInterface::plugin, tr("Add a cylinder"), QString::number(cyl->getRadius()));
            }
        }
    }
    //Estimate cylinder
}
