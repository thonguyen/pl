#include "lb_stepdetectdefectspcl.h"

#include <math.h>
#include <cmath>
#include<float.h>

#include <QFile>
#include <QTextStream>
#include <QDateTime>

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"

#include "ct_itemdrawable/ct_cylinder.h"
#include "ct_itemdrawable/ct_pointcluster.h"
#include "ct_itemdrawable/ct_scene.h"
#include "ct_itemdrawable/ct_meshmodel.h"

#include "ct_result/tools/iterator/ct_resultgroupiterator.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_iterator/ct_mutablepointiterator.h"

#include "ct_result/ct_resultgroup.h"
#include "ct_view/ct_stepconfigurabledialog.h"

#include "ct_mesh/ct_mesh.h"
#include "ct_mesh/tools/ct_meshallocator.h"


#include "pcl/common/common.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/cloud_iterator.h>
#include <pcl/segmentation/extract_clusters.h>


#define DEF_IN_RESULT               "InResult"
#define DEF_IN_GROUP                "InGroup"
#define DEF_IN_POINT_CLOUD          "InPoints"
#define DEF_IN_SCENE                "InScene"
#define DEF_OUT_RESULT              "OutResult"
#define DEF_OUT_GROUP               "OutGroup"
#define DEF_OUT_CLUSTER             "OutCluster"
#define DEF_OUT_GROUP_CLUSTER       "OutGroupCluster"
#define DEF_OUT_CIRCLE              "OutCircle"
#define DEF_OUT_SCENE               "OutScene"
#define DEF_OUT_CYLINDER            "OutCylinder"
#define DEF_OUT_CYLINDER_PCL        "OutCylinderPCL"
#define DEF_OUT_MESH                "OutMesh"


#define MINIMUM_DEFAULT_SIZE 4

// Constructor : initialization of parameters
LB_StepDetectDefectsPcl::LB_StepDetectDefectsPcl(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

// Step description (tooltip of contextual menu)
QString LB_StepDetectDefectsPcl::getStepDescription() const
{
    return tr("Detect external defects");
}

// Step detailled description
QString LB_StepDetectDefectsPcl::getStepDetailledDescription() const
{
    return tr("Detect external defects using PCL");
}

// Step URL
QString LB_StepDetectDefectsPcl::getStepURL() const
{
    //return tr("STEP URL HERE");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* LB_StepDetectDefectsPcl::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new LB_StepDetectDefectsPcl(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void LB_StepDetectDefectsPcl::createInResultModelListProtected()
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
void LB_StepDetectDefectsPcl::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEF_IN_RESULT);
    res->setRootGroup(DEF_OUT_GROUP, new CT_StandardItemGroup(), tr("Groupe"));
    res->addGroupModel(DEF_OUT_GROUP,outGroupClusterModelName, new CT_StandardItemGroup(), tr("Groupe de clusters"));
    res->addItemModel(DEF_OUT_GROUP, outSceneModelName, new CT_Scene(), tr("Scène(s) transformée(s)"));
    res->addItemModel(DEF_OUT_GROUP, outCylinderModelName, new CT_Cylinder(), tr("Cylindre"));
    res->addItemModel(DEF_OUT_GROUP_CLUSTER, outClusterModelName, new CT_PointCluster, tr("Cluster(s)"));

/*
    CT_OutResultModelGroup* res = createNewOutResultModel(DEF_OUT_RESULT, tr("Scène(s) transformée(s)"));
    res->setRootGroup(DEF_OUT_GROUP, new CT_StandardItemGroup(), tr("Groupe"));
    res->addItemModel(DEF_OUT_GROUP, DEF_OUT_SCENE, new CT_Scene(), tr("Scène(s) transformée(s)"));
    res->addItemModel(DEF_OUT_GROUP, DEF_OUT_CYLINDER, new CT_Cylinder(), tr("Cylindre"));
    res->addItemModel(DEF_OUT_GROUP, DEF_OUT_CYLINDER_PCL, new CT_Cylinder(), tr("Cylindre estimated by PCL"));
    res->addItemModel(DEF_OUT_GROUP, DEF_OUT_MESH, new CT_MeshModel(), tr("Plane"));
    res->addGroupModel(DEF_OUT_GROUP, DEF_OUT_GROUP_CLUSTER, new CT_StandardItemGroup(), tr("Clusters de défaults"));
    for(int i = 0; i < 100; i++){
        QString modelName = DEF_OUT_CLUSTER + QString::number(i);
        res->addItemModel(DEF_OUT_GROUP_CLUSTER, modelName, new CT_PointCluster, tr("Cluster"));
    }
    */
}

// Semi-automatic creation of step parameters DialogBox
void LB_StepDetectDefectsPcl::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog* dialog = newStandardPostConfigurationDialog();
    dialog->addDouble(tr("Slice length"), "m", 0.1, 1.0, 4, sliceLength);
    dialog->addDouble(tr("Distance threshold"), "m", 0.0, 0.05, 4, distanceThreshold);
    dialog->addDouble(tr("Angle threshold"), "rad", 0.0, 1.55, 4, angleThreshold);
}

void LB_StepDetectDefectsPcl::compute()
{
    // IN results browsing
    CT_ResultGroup* resultGroup = getInputResults().first();
    CT_ResultGroup* outResult = (CT_ResultGroup*) getOutResultList().first();
    //CT_StandardItemGroup* outGroup = new CT_StandardItemGroup(DEF_OUT_GROUP, outResult);


    //CT_ResultGroupIterator it(outsult, this, DEF_IN_GROUP);

    CT_ResultGroupIterator it(resultGroup, this, DEF_IN_GROUP);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while(it.hasNext() && !isStopped()){
        CT_AbstractItemGroup* group = (CT_AbstractItemGroup*)it.next();

        const CT_AbstractItemDrawableWithPointCloud* item =
                (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, DEF_IN_POINT_CLOUD);
        if(item != NULL)
        {
            // on récupère le nuage d'indices de la scène
            CT_PCIR pcir = item->getPointCloudIndexRegistered();
            //CT_PointCluster * pointCluster = new CT_PointCluster(_outClusterModelName.completeName(), outResult);

            // création de l'itérateur
            CT_PointIterator itPoint(pcir);            
            while(itPoint.hasNext() && !isStopped()){
                itPoint.next();
                // récupération du point
                const CT_Point &p = itPoint.currentPoint();                
                cloud->points.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
            }
        }

        //preprocessPointCloud(inputCloud, cloud);
        CT_Scene *outScene = new CT_Scene(DEF_OUT_SCENE, outResult);

        //transformCloud(*cloud, transMat, outScene);
        transformCloudByEnertia(*cloud, outScene);
        CT_StandardItemGroup *outGroup = new CT_StandardItemGroup(DEF_OUT_GROUP, outResult);
        CT_StandardItemGroup *outGroupCluster = new CT_StandardItemGroup(DEF_OUT_GROUP_CLUSTER, outResult);
        outGroup->addGroup(outGroupCluster);
        outGroup->addItemDrawable(outScene);
        outResult->addGroup(outGroup);

        std::vector<pcl::PointIndices> sliceIndices;
        cutLog(*cloud, sliceLength, sliceIndices);

        double angleStep = 2*M_PI/20;
        std::vector<pcl::PointIndices::Ptr> cloudPartIndices;
        segmentByAngle(cloud, sliceIndices, angleStep, cloudPartIndices);

        pcl::PointIndices::Ptr defaultIndices (new pcl::PointIndices);
        std::vector<pcl::ModelCoefficients::Ptr> planesCoefficients = detectDefaults(cloud, cloudPartIndices, distanceThreshold, *defaultIndices);

        std::vector<pcl::PointIndices> clusterIndices;
        clusterPointCloud(cloud, defaultIndices, clusterIndices);
        int clusterIndex = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); it++)
        {

            if(it->indices.size() > MINIMUM_DEFAULT_SIZE)
            {
                QString modelName = DEF_OUT_CLUSTER + QString::number(clusterIndex);
                clusterIndex++;
                CT_PointCluster* pointCluster= new CT_PointCluster(modelName, outResult);
                const CT_AbstractPointCloudIndex* indices = outScene->getPointCloudIndex();
                int offset = indices->first();
                //Q_ASSERT(indices.size() == it->indices.size());
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                {
                    pointCluster->addPoint(offset + *pit);
                }
                outGroupCluster->addItemDrawable(pointCluster);
                qDebug()<<"Cluster size: "<<pointCluster->getPointCloudIndexSize();
            }
        }

        qDebug() << "Number of points" << cloud->points.size();
        //qDebug() << "Number of defect points" << defautls->points.size();

        double rad = 0.08;
        double h = 1.0;
        CT_CylinderData *cylinderData = new CT_CylinderData(Eigen::Vector3d(0,0,0), Eigen::Vector3d::UnitX(), rad, h);
        if(cylinderData != NULL && cylinderData->getRadius() > 0)
        {
            CT_Cylinder *cyl = new CT_Cylinder(DEF_OUT_CYLINDER, outResult, cylinderData);
            outGroup->addItemDrawable(cyl);
            PS_LOG->addMessage(LogInterface::debug, LogInterface::plugin, tr("Add a cylinder with radius = "), "");
            PS_LOG->addMessage(LogInterface::debug, LogInterface::plugin, QString::number(cyl->getRadius()));
        }

        pcl::ModelCoefficients::Ptr  coefficientsCylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliersCylinder (new pcl::PointIndices);

        coefficientsCylinder = estimateCylinder(cloud, *inliersCylinder);
        Eigen::Vector3d onePointInDirection(coefficientsCylinder->values[0], coefficientsCylinder->values[1], coefficientsCylinder->values[2]);
        Eigen::Vector3d directionPcl(coefficientsCylinder->values[3], coefficientsCylinder->values[4], coefficientsCylinder->values[5]);

        CT_CylinderData *cylinderDataPcl = new CT_CylinderData(onePointInDirection, directionPcl, coefficientsCylinder->values[6], h);
        if(cylinderData != NULL && cylinderDataPcl->getRadius() > 0)
        {
            CT_Cylinder *cyl = new CT_Cylinder(DEF_OUT_CYLINDER_PCL, outResult, cylinderDataPcl);
            outGroup->addItemDrawable(cyl);
            PS_LOG->addMessage(LogInterface::debug, LogInterface::plugin, tr("Add a cylinder with radius = "), "");
            PS_LOG->addMessage(LogInterface::debug, LogInterface::plugin, QString::number(cyl->getRadius()));
        }

        //add a plane for testing
        double maxX = -DBL_MAX;
        double minX = DBL_MAX;
        //find the max and min of x
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            minX = cloud->points[i].x < minX ? cloud->points[i].x : minX;
            maxX = cloud->points[i].x > maxX ? cloud->points[i].x : maxX;
        }
        pcl::ModelCoefficients firstPlane = *planesCoefficients[0];
        //y = rad
        double y1 = rad;
        double y2 = rad - 5;
        double a = firstPlane.values[0];
        double b = firstPlane.values[1];
        double c = firstPlane.values[2];
        double d = firstPlane.values[3];
        double z1 = (-d - a*minX - b*y1) / c;
        double z2 = (-d - a*maxX - b*y1) / c;
        double z3 = (-d - a*minX - b*y2) / c;
        double z4 = (-d - a*maxX - b*y2) / c;

        // création du mesh
        CT_Mesh *mesh = new CT_Mesh();
        // création de 4 points
        CT_MutablePointIterator itP = CT_MeshAllocator::AddVertices(mesh, 4);
        // passe au premier point
        itP.next();
        // on récupère son index global
        size_t globalIndexFirstPoint = itP.currentGlobalIndex();

        // modification du point 1
        itP.replaceCurrentPoint(createCtPoint(minX, y1, z1));

        // modification du point 2 en une seule ligne
        itP.next().replaceCurrentPoint(createCtPoint(maxX, y1, z2));

        // modification du point 3 en une seule ligne
        itP.next().replaceCurrentPoint(createCtPoint(minX, y2, z3));

        // modification du point 4 en une seule ligne
        itP.next().replaceCurrentPoint(createCtPoint(maxX, y2, z4));

        /******************************/
        /*********** FACES ************/
        /******************************/

        // création de 2 faces
        CT_MutableFaceIterator itF = CT_MeshAllocator::AddFaces(mesh, 2);

        // récupère la première face
        CT_Face &face1 = itF.next().cT();

        // récupère l'index de la face 1
        size_t faceIndex = itF.cIndex();

        // création de 3 arêtes
        CT_MutableEdgeIterator itE = CT_MeshAllocator::AddHEdges(mesh, 3);

        // création de variable contenant les indices des arêtes
        size_t e1Index = itE.next().cIndex();
        size_t e2Index = e1Index + 1;
        size_t e3Index = e1Index + 2;

        // on passe l'index de la première arête à la face. Une face doit connaitre au moins une arête.
        face1.setEdge(e1Index);

        /******************************/
        /*********** ARETES ***********/
        /******* première partie ******/
        /******************************/

        // création de variables contenant les indices des points de la face
        size_t p0 = globalIndexFirstPoint;
        size_t p1 = p0+1;
        size_t p2 = p0+2;

        // on récupère l'arête 1
        CT_Edge &e1 = itE.next().cT();

        // et on lui affecte les indices des points qui la compose
        e1.setPoint0(p0);
        e1.setPoint1(p1);

        // ainsi que l'indice de la face
        e1.setFace(faceIndex);

        // on récupère l'arête 2
        CT_Edge &e2 = itE.next().cT();
        e2.setPoint0(p1);
        e2.setPoint1(p2);
        e2.setFace(faceIndex);

        // on récupère l'arête 3
        CT_Edge &e3 = itE.next().cT();
        e3.setPoint0(p2);
        e3.setPoint1(p0);
        e3.setFace(faceIndex);

        // on définit qui est la précédente, qui est la suivante parmis les arêtes
        e1.setNext(e2Index);
        e1.setPrevious(e3Index);
        e2.setNext(e3Index);
        e2.setPrevious(e1Index);
        e3.setNext(e1Index);
        e3.setPrevious(e2Index);

        /******************************/
        /*********** ARETES ***********/
        /******* deuxième partie ******/
        /******************************/

        // création de 3 nouvelles arêtes
        itE = CT_MeshAllocator::AddHEdges(mesh, 3);

        // création de variable contenant les indices des arêtes
        e1Index = itE.next().cIndex();
        e2Index = e1Index + 1;
        e3Index = e1Index + 2;

        // récupère la deuxième face
        CT_Face &face2 = itF.next().cT();

        // récupère l'index de la face 2
        faceIndex = itF.cIndex();

        // on passe l'index de la première arête à la face. Une face doit connaitre au moins une arête.
        face2.setEdge(e1Index);

        // création de variables contenant les indices des points de la face
        p0 = globalIndexFirstPoint+3;
        p1 = p0+1;
        p2 = p0+2;

        // on récupère l'arête 1
        CT_Edge &e12 = itE.next().cT();

        // et on lui affecte les indices des points qui la compose
        e12.setPoint0(p0);
        e12.setPoint1(p1);

        // ainsi que l'indice de la face
        e12.setFace(faceIndex);

        // on récupère l'arête 2
        CT_Edge &e22 = itE.next().cT();
        e22.setPoint0(p1);
        e22.setPoint1(p2);
        e22.setFace(faceIndex);

        // on récupère l'arête 3
        CT_Edge &e32 = itE.next().cT();
        e32.setPoint0(p2);
        e32.setPoint1(p0);
        e32.setFace(faceIndex);

        // on définit qui est la précédente, qui est la suivante parmis les arêtes
        e12.setNext(e2Index);
        e12.setPrevious(e3Index);
        e22.setNext(e3Index);
        e22.setPrevious(e1Index);
        e32.setNext(e1Index);
        e32.setPrevious(e2Index);


        // création du CT_MeshModel
        CT_MeshModel *meshModel  = new CT_MeshModel(DEF_OUT_MESH, outResult, mesh);
        outGroup->addItemDrawable( meshModel);

    }
}

void LB_StepDetectDefectsPcl::segmentByAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                             std::vector<pcl::PointIndices> sliceIndices,
                                             double angleStep,
                                             std::vector<pcl::PointIndices::Ptr> &cloudPartIndices)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif
    int nbArc = ceil(2 * M_PI / angleStep);
    for(size_t i = 0; i < nbArc * sliceIndices.size(); i++)
    {
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPart(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr cloudPart (new pcl::PointIndices);
        cloudPartIndices.push_back(cloudPart);        
    }

    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (10);
    ne.compute (*cloudNormals);

    for (size_t sliceId = 0; sliceId < sliceIndices.size(); sliceId++)
    {
        if(sliceIndices[sliceId].indices.size() < 100)
        {
            continue;
        }
        /*
        pcl::ModelCoefficients::Ptr  coefficientsCylinder (new pcl::ModelCoefficients);

        coefficientsCylinder = estimateCylinderOfSlice(cloud, cloudNormals, sliceIndices[sliceId]);
        if(coefficientsCylinder->values.size() == 0){
            continue;
        }

        Eigen::Vector3d onePointInDirection(coefficientsCylinder->values[0], coefficientsCylinder->values[1], coefficientsCylinder->values[2]);
        Eigen::Vector3d directionPcl(coefficientsCylinder->values[3], coefficientsCylinder->values[4], coefficientsCylinder->values[5]);

        Eigen::Matrix4d transMat = getTransformationMatrix(onePointInDirection, directionPcl);

        transformCloud(*cloud, sliceIndices[sliceId], transMat);
        */
        for (size_t i = 0; i < sliceIndices[sliceId].indices.size(); i++)
        {
            pcl::PointXYZ p = cloud->points[sliceIndices[sliceId].indices.at(i)];
            /*
            //distance to center vector of cylinder
            Eigen::Vector3d p0(onePointInDirection.x() - p.x, onePointInDirection.y() - p.y, onePointInDirection.z() - p.z);
            Eigen::Vector3d v = p0.cross(directionPcl);
            double d = v.norm()/directionPcl.norm();
            //don't include points that are close to center
            if(d < 0.7*coefficientsCylinder->values[6])
            {
                continue;
            }
            */
            //Calculate angle with Oy
            double angle = acos(p.y/sqrt( pow(p.y,2) + pow(p.z,2)));
            if(p.z < 0)
            {
                angle = 2 * M_PI - angle;
            }
            int segment = sliceId*nbArc + angle / angleStep;
            cloudPartIndices[segment]->indices.push_back(sliceIndices[sliceId].indices[i]);
        }
    }
#ifdef QT_DEBUG
  qint64 end = QDateTime::currentMSecsSinceEpoch();
  qDebug()<<"Segmentation in: "<< end - begin;
#endif

}

std::vector<pcl::ModelCoefficients::Ptr>
LB_StepDetectDefectsPcl::detectDefaults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        std::vector<pcl::PointIndices::Ptr> cloudPartIndices,
                                        double threshold,
                                        pcl::PointIndices &defautls)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    QString filename = "Error" + QString::number(currentTime);
    QFile file(filename);

    file.open(QIODevice::ReadWrite);
    QTextStream stream(&file);

    double cosNormalThreshold = cos(angleThreshold);
    std::vector<pcl::ModelCoefficients::Ptr> planesCoefficients;
    for(size_t i = 0; i < cloudPartIndices.size(); i++)
    {
        if(cloudPartIndices[i]->indices.size() == 0){
            continue;
        }
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud);
        ne.setIndices(cloudPartIndices[i]);
        ne.setKSearch (10);
        ne.compute (*cloudNormals);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.003);
        seg.setInputCloud (cloud);
        seg.setIndices(cloudPartIndices[i]);
        seg.segment (*inliers, *coefficients);

        planesCoefficients.push_back(coefficients);

        //@TODO:clean //for debug purpose
        int nbPoints = cloudPartIndices[i]->indices.size();
        int nbDefaults = 0;

        for (size_t j = 1; j < cloudPartIndices[i]->indices.size(); j++)
        {
            int index = cloudPartIndices[i]->indices[j];
            double distance = std::abs(coefficients->values[0]*cloud->points[index].x +
                                  coefficients->values[1]*cloud->points[index].y +
                                  coefficients->values[2]*cloud->points[index].z +
                                  coefficients->values[3]);
            stream <<cloud->points[index].x<<"\t"<<cloud->points[index].y<<"\t"<<cloud->points[index].z<<"\t" << distance<<"\t"<<"\n";
            if(distance > threshold )
            {
                pcl::Normal n = cloudNormals->points[i];
                //Compute angle between point normal and plane normal
                double cosTheta = n.normal_x * coefficients->values[0] + n.normal_y * coefficients->values[1] + n.normal_z * coefficients->values[2];
                if(cosTheta < cosNormalThreshold)
                {
                    defautls.indices.push_back(index);
                    //@TODO:clean //for debug purpose
                    nbDefaults++;
                }
            }
        }
        qDebug() << "Cloud part " << i << ", defaults/all: " << nbDefaults << "/"<< nbPoints;
    }
    file.close();
#ifdef QT_DEBUG
    qint64 end = QDateTime::currentMSecsSinceEpoch();
    qDebug()<<"Detection in: "<< end - begin;
#endif

    return planesCoefficients;
}

void LB_StepDetectDefectsPcl::clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                pcl::PointIndices::Ptr defaultIndices,
                                                std::vector<pcl::PointIndices> &clusterIndices)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd (new pcl::search::KdTree<pcl::PointXYZ>);
    kd->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.005);
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (cloud->points.size());
    ec.setSearchMethod (kd);
    ec.setInputCloud (cloud);
    ec.setIndices(defaultIndices);
    ec.extract (clusterIndices);
#ifdef QT_DEBUG
    qint64 end = QDateTime::currentMSecsSinceEpoch();
    qDebug()<<"Cluster in: "<< end - begin;
#endif

}

Eigen::Matrix4d LB_StepDetectDefectsPcl::getTransformationMatrix(Eigen::Vector3d directionVector,
                                                                 Eigen::Vector3d translationVector){
    directionVector.normalize();
    double a = directionVector[0];
    double b = directionVector[1];
    double c = directionVector[2];

    double d = sqrt(a*a + c*c);
    //Rotate direction vector about Oy, after this rotation, direction vector is in Oxy
    Eigen::Matrix4d rotationY = Eigen::Matrix4d::Identity();
    if(d != 0){
        //cos of the rotation angle theta
        double ct = a / d;
        //sin of the rotation angle theta
        double st = c / d;
        rotationY(0,0) = ct;
        rotationY(0,2) = st;
        rotationY(2,0) = -st;
        rotationY(2,2) = ct;
    }

    //Rotate direction vector about Oz, after this rotation, direction vector is Ox aligned
    Eigen::Matrix4d rotationZ = Eigen::Matrix4d::Identity();
    //sin(phi) = b, cos(phi) = d
    rotationZ(0,0) = d;
    rotationZ(0,1) = -b;
    rotationZ(1,0) = b;
    rotationZ(1,1) = d;

    Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
    translation(0,3) = translationVector[0];
    translation(1,3) = translationVector[1];
    translation(2,3) = translationVector[2];

    return rotationZ * rotationY*translation;
}

void LB_StepDetectDefectsPcl::transformCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Matrix4d transMat, CT_Scene *outScene)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif
    // limites de la bounding-box de la scène de sortie
    double minX = std::numeric_limits<double>::max();
    double minY = minX;
    double minZ = minX;

    double maxX = -minX;
    double maxY = -minX;
    double maxZ = -minX;

    CT_NMPCIR pcir = PS_REPOSITORY->createNewPointCloud(cloud.size());

    // create a mutable point iterator to change points of this cloud
    CT_MutablePointIterator outItP(pcir);
    int i = 0;
    while(outItP.hasNext())
    {
        CT_Point point;
        Eigen::Vector4d v(cloud[i].x, cloud[i].y, cloud[i].z, 1);
        Eigen::Vector4d transformed = transMat*v;
        point.setX(transformed[0]);
        point.setY(transformed[1]);
        point.setZ(transformed[2]);
        // set it to the new point cloud
        outItP.next().replaceCurrentPoint(point);
        pcl::PointXYZ pointPcl(transformed[0], transformed[1], transformed[2]);
        cloud[i] = pointPcl;
        if (point(0) < minX) {minX = point(0);}
        if (point(1) < minY) {minY = point(1);}
        if (point(2) < minZ) {minZ = point(2);}
        if (point(0) > maxX) {maxX = point(0);}
        if (point(1) > maxY) {maxY = point(1);}
        if (point(2) > maxZ) {maxZ = point(2);}
        i++;
    }

    outScene->setPointCloudIndexRegistered(pcir);
    outScene->setBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
#ifdef QT_DEBUG
    qint64 end = QDateTime::currentMSecsSinceEpoch();
    qDebug()<<"Transform in: "<< end - begin;
#endif
}

void LB_StepDetectDefectsPcl::transformCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointIndices indices, Eigen::Matrix4d transMat)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif


    for(size_t i; i < indices.indices.size(); i++){
        Eigen::Vector4d v(cloud[indices.indices[i]].x, cloud[indices.indices[i]].y, cloud[indices.indices[i]].z, 1);
        Eigen::Vector4d transformed = transMat*v;
        pcl::PointXYZ pointPcl(transformed[0], transformed[1], transformed[2]);
        cloud[i] = pointPcl;
    }
#ifdef QT_DEBUG
    qint64 end = QDateTime::currentMSecsSinceEpoch();
    qDebug()<<"Transform slice in: "<< end - begin;
#endif
}

void LB_StepDetectDefectsPcl::transformCloudByEnertia(pcl::PointCloud<pcl::PointXYZ> &cloud, CT_Scene *outScene)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif
    //Lecture du nuage
    Eigen ::Vector3d centroid = getCentroid(cloud);
    //pcl::compute3DCentroid(cloud, centroid);

    //Calcul des coeffcients de la matrice d'inertie
    double x2 = 0.0;
    double y2 = 0.0;
    double z2 = 0.0;
    double xy = 0.0;
    double yz = 0.0;
    double xz = 0.0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for(size_t i=0; i < cloud.size() ; i++)
    {
        cloud[i].x = cloud[i].x - centroid[0];
        cloud[i].y = cloud[i].y - centroid[1];
        cloud[i].z = cloud[i].z - centroid[2];

        x2 += pow(cloud[i].x,2);
        y2 += pow(cloud[i].y,2);
        z2 += pow(cloud[i].z,2);

        yz += cloud[i].y * cloud[i].z;
        xz += cloud[i].x * cloud[i].z;
        xy += cloud[i].x * cloud[i].y;

        sumX += cloud[i].x;
        sumY += cloud[i].y;
        sumZ += cloud[i].z;
    }

    double Ixx = y2 + z2;
    double Iyy = x2 + z2;
    double Izz = x2 + y2;
    double Ixy = -xy;
    double Ixz = -xz;
    double Iyz = -yz;

    // MI matrice d'inertie
    Eigen::Matrix3d MI;

    MI << Ixx, Ixy, Ixz,
          Ixy, Iyy, Iyz,
          Ixz, Iyz, Izz;

    // Calcul des valeurs propres et vecteurs propres

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    //Eigen::EigenSolver<Eigen::Matrix3d> eigensolver(MI);
    Eigen::Matrix3d eigenVectors;
    Eigen::Vector3d lambda;
    eigensolver.compute(MI);
    lambda = eigensolver.eigenvalues();
    eigenVectors = eigensolver.eigenvectors();

    Eigen::Vector3d V3;
    Eigen::Vector3d V2;
    Eigen::Vector3d V1;

    //produit vectoriel de v1 et v2
    V3(0) = eigenVectors.col(0)(1) * eigenVectors.col(1)(2) - eigenVectors.col(0)(2) * eigenVectors.col(1)(1);
    V3(1) = eigenVectors.col(1)(0) * eigenVectors.col(0)(2) - eigenVectors.col(1)(2) * eigenVectors.col(0)(0);
    V3(2) = eigenVectors.col(0)(0) * eigenVectors.col(1)(1) - eigenVectors.col(0)(1) * eigenVectors.col(1)(0);


    //produit vectoriel de v2 et v3
    V1(0) = eigenVectors.col(2)(1) * eigenVectors.col(1)(2) - eigenVectors.col(2)(2) * eigenVectors.col(1)(1);
    V1(1) = eigenVectors.col(1)(0) * eigenVectors.col(2)(2) - eigenVectors(2,1) * eigenVectors.col(2)(0);
    V1(2) = eigenVectors.col(2)(0) * eigenVectors.col(1)(1) - eigenVectors.col(2)(1) * eigenVectors.col(1)(0);

    //produit vectoriel de v1 et v3
    V2(0)= eigenVectors.col(2)(1) * eigenVectors.col(0)(2) - eigenVectors.col(2)(2) * eigenVectors.col(0)(1);
    V2(1)= eigenVectors.col(0)(0) * eigenVectors.col(2)(2) - eigenVectors.col(0)(2) * eigenVectors.col(2)(0);
    V2(2)= eigenVectors.col(2)(0) * eigenVectors.col(0)(1) - eigenVectors.col(2)(1) * eigenVectors.col(0)(0);

    //A VERIFIER
    if( eigenVectors(2,0) < 0)
    {
        eigenVectors.col(0) = -eigenVectors.col(0);

        if(V3(0)*eigenVectors(0,2) >0 && V3(1)*eigenVectors(1,2) >0 && V3(2) * eigenVectors(2,2) > 0)
            eigenVectors.col(1) = - eigenVectors.col(1);

        if(V2(0)* eigenVectors(0,1) > 0 && V2(1)* eigenVectors(1,1) > 0 && V2(2)* eigenVectors(2,1) > 0)
            eigenVectors.col(2) = - eigenVectors.col(2);
    }else
    {
        //X.col(1) = - X.col(1);
        eigenVectors.col(2) = - eigenVectors.col(2);
    }

    // limites de la bounding-box de la scène de sortie
    double minX = std::numeric_limits<double>::max();
    double minY = minX;
    double minZ = minX;

    double maxX = -minX;
    double maxY = -minX;
    double maxZ = -minX;

    CT_NMPCIR pcir = PS_REPOSITORY->createNewPointCloud(cloud.size());

    // create a mutable point iterator to change points of this cloud
    CT_MutablePointIterator outItP(pcir);
    int i = 0;
    Eigen::Matrix3d rot = eigenVectors.transpose();
    while(outItP.hasNext())
    {
        CT_Point point;
        Eigen::Vector3d v(cloud[i].x, cloud[i].y, cloud[i].z);
        Eigen::Vector3d transformed = rot*v;
        point.setX(transformed[0]);
        point.setY(transformed[1]);
        point.setZ(transformed[2]);
        // set it to the new point cloud
        outItP.next().replaceCurrentPoint(point);
        pcl::PointXYZ pointPcl(transformed[0], transformed[1], transformed[2]);
        cloud[i] = pointPcl;
        if (point(0) < minX) {minX = point(0);}
        if (point(1) < minY) {minY = point(1);}
        if (point(2) < minZ) {minZ = point(2);}
        if (point(0) > maxX) {maxX = point(0);}
        if (point(1) > maxY) {maxY = point(1);}
        if (point(2) > maxZ) {maxZ = point(2);}
        i++;
    }

    outScene->setPointCloudIndexRegistered(pcir);
    outScene->setBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
#ifdef QT_DEBUG
    qint64 end = QDateTime::currentMSecsSinceEpoch();
    qDebug()<<"Transform in: "<< end - begin;
#endif
}

pcl::ModelCoefficients::Ptr
LB_StepDetectDefectsPcl::estimateCylinderOfSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
                                                 pcl::PointIndices pointIndices)
{
    pcl::ModelCoefficients::Ptr  coefficientsCylinder (new pcl::ModelCoefficients);
    //Estimate cylinder
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices inliersCylinder;
    pcl::PointIndices::Ptr pi(new pcl::PointIndices);
    pi->indices = pointIndices.indices;
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.001);
    seg.setAxis(Eigen::Vector3f::UnitX());
    seg.setEpsAngle(0.01);
    //seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.005);
    seg.setRadiusLimits (0.05, 0.15);
    seg.setInputCloud (cloud);
    seg.setIndices(pi);
    seg.setInputNormals (cloudNormals);
    seg.segment (inliersCylinder, *coefficientsCylinder);

    return coefficientsCylinder;

}



pcl::ModelCoefficients::Ptr
LB_StepDetectDefectsPcl::estimateCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointIndices &inliersCylinder)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr  coefficientsCylinder (new pcl::ModelCoefficients);
    //Estimate cylinder
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ne.compute (*cloudNormals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.001);
    seg.setAxis(Eigen::Vector3f::UnitX());
    seg.setEpsAngle(0.01);
    //seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.005);
    seg.setRadiusLimits (0.05, 0.15);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloudNormals);
    seg.segment (inliersCylinder, *coefficientsCylinder);

    return coefficientsCylinder;
}

void LB_StepDetectDefectsPcl::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud)
{
#ifdef QT_DEBUG
    qint64 begin = QDateTime::currentMSecsSinceEpoch();
#endif
    //éliminer les points fantômes
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.01);
    outrem.setMinNeighborsInRadius (2);

    // apply filter
    outrem.filter (*outputCloud);
#ifdef QT_DEBUG
    qint64 end = QDateTime::currentMSecsSinceEpoch();
    qDebug()<<"Preprocess in: "<< end - begin;
#endif
}

void LB_StepDetectDefectsPcl::cutLog(pcl::PointCloud<pcl::PointXYZ>cloud,
                                    double sliceLength,
                                    std::vector<pcl::PointIndices> &sliceIndices)
{
    double maxX = -DBL_MAX;
    double minX = DBL_MAX;
    //find the max and min of x

    for(size_t i = 0; i < cloud.size(); i++)
    {
        minX = cloud[i].x < minX ? cloud[i].x : minX;
        maxX = cloud[i].x > maxX ? cloud[i].x : maxX;
    }

    double logLength = maxX - minX;
    int nbSlice = ceil(logLength / sliceLength);

    for(int i = 0; i < nbSlice; i++)
    {
        pcl::PointIndices::Ptr slice(new pcl::PointIndices);
        sliceIndices.push_back(*slice);
    }

    for(size_t i = 0; i < cloud.size(); i++)
    {
        //remove 5mm at extremity
        if(cloud[i].x > maxX - 0.005 ){
            continue;
        }
        int sliceIndex = (cloud[i].x - minX)/ sliceLength;
        sliceIndices[sliceIndex].indices.push_back(i);
    }
}


void LB_StepDetectDefectsPcl::sceneToPCLCloud(CT_Scene *scene, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    CT_PCIR pcir = scene->getPointCloudIndexRegistered();

    // création de l'itérateur
    CT_PointIterator itPoint(pcir);
    while(itPoint.hasNext() && !isStopped()){
        itPoint.next();
        // récupération du point
        const CT_Point &p = itPoint.currentPoint();
        cloud.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
}

Eigen::Matrix3d LB_StepDetectDefectsPcl::getEigenVectors(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    //Calcul des coeffcients de la matrice d'inertie
    double x2 = 0.0;
    double y2 = 0.0;
    double z2 = 0.0;
    double xy = 0.0;
    double yz = 0.0;
    double xz = 0.0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for(size_t i=0; i < cloud.size() ; i++)
    {
        x2 += pow(cloud[i].x,2);
        y2 += pow(cloud[i].y,2);
        z2 += pow(cloud[i].z,2);

        yz += cloud[i].y * cloud[i].z;
        xz += cloud[i].x * cloud[i].z;
        xy += cloud[i].x * cloud[i].y;

        sumX += cloud[i].x;
        sumY += cloud[i].y;
        sumZ += cloud[i].z;
    }

    double Ixx = y2 + z2 - (sumY*sumY + sumZ*sumZ)/cloud.size();
    double Iyy = x2 + z2 - (sumX*sumX + sumZ*sumZ)/cloud.size();
    double Izz = x2 + y2 - (sumX*sumX + sumY*sumY)/cloud.size();
    double Ixy = -xy + sumX*sumY/cloud.size();
    double Ixz = -xz + sumX*sumZ/cloud.size();
    double Iyz = -yz + sumY*sumZ/cloud.size();

    // MI matrice d'inertie
    Eigen::Matrix3d MI;

    MI << Ixx, Ixy, Ixz,
          Ixy, Iyy, Iyz,
          Ixz, Iyz, Izz;

    // Calcul des valeurs propres et vecteurs propres

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    //Eigen::EigenSolver<Eigen::Matrix3d> eigensolver(MI);
    Eigen::Matrix3d eigenVectors;
    Eigen::Vector3d lambda;
    eigensolver.compute(MI);
    lambda = eigensolver.eigenvalues();
    eigenVectors = eigensolver.eigenvectors();
    return eigenVectors;
}

Eigen::Vector3d LB_StepDetectDefectsPcl::getCentroid(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    double x = 0;
    double y = 0;
    double z = 0;
    for (size_t i = 0; i < cloud.size(); i++)
    {
        x += cloud[i].x;
        y += cloud[i].y;
        z += cloud[i].z;
    }
    return Eigen::Vector3d(x / cloud.size(), y / cloud.size(), z / cloud.size());
}
