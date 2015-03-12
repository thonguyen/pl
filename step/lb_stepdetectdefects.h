#ifndef LB_STEPDETECTDEFECTS_H
#define LB_STEPDETECTDEFECTS_H

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_tools/model/ct_autorenamemodels.h"

#include "ct_itemdrawable/ct_pointcluster.h"
/*!
 * \class LB_StepDetectDefects
 * \ingroup Steps_LB
 * \brief <b>Detect log defects.</b>
 *
 * Detect log defects
 *
 *
 */

class LB_StepDetectDefects: public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     * 
     * Create a new instance of the step
     * 
     * \param dataInit Step parameters object
     */
    LB_StepDetectDefects(CT_StepInitializeData &dataInit);

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

    void estimateCylinder(CT_PointCluster * pointCluster);

private:
    CT_AutoRenameModels     _outCylinderModelName;
    CT_AutoRenameModels     _outClusterModelName;
    // Step parameters
    // No parameter for this step
};

#endif // LB_STEPDETECTDEFECTS_H
