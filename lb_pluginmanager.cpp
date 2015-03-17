#include "lb_pluginmanager.h"
#include "ct_stepseparator.h"
#include "ct_steploadfileseparator.h"
#include "ct_stepcanbeaddedfirstseparator.h"
#include "ct_actions/ct_actionsseparator.h"
#include "ct_exporter/ct_standardexporterseparator.h"
#include "ct_reader/ct_standardreaderseparator.h"
#include "ct_actions/abstract/ct_abstractaction.h"

// Inclure ici les entetes des classes definissant des Ã©tapes/actions/exporters ou readers
#include "step/lb_stepcharacterisecurvature.h"
#include "step/lb_stepcharacterizelog.h"
#include "step/lb_stepdetectdefects.h"
#include "step/lb_stepdetectdefectspcl.h"

LB_PluginManager::LB_PluginManager() : CT_AbstractStepPlugin()
{
}

LB_PluginManager::~LB_PluginManager()
{
}

bool LB_PluginManager::loadGenericsStep()
{
    CT_StepSeparator *sep = addNewSeparator(new CT_StepSeparator());
    // Ajouter ici les etapes
    //sep->addStep(new NomDeLEtape(*createNewStepInitializeData(NULL)));
    sep->addStep(new LB_StepCharacteriseCurvature(*createNewStepInitializeData(NULL)));
    sep->addStep(new LB_StepCharacterizeLog(*createNewStepInitializeData(NULL)));
    sep->addStep(new LB_StepDetectDefects(*createNewStepInitializeData(NULL)));
    sep->addStep(new LB_StepDetectDefectsPcl(*createNewStepInitializeData(NULL)));

    return true;
}

bool LB_PluginManager::loadOpenFileStep()
{
    clearOpenFileStep();

    CT_StepLoadFileSeparator *sep = addNewSeparator(new CT_StepLoadFileSeparator(("TYPE DE FICHIER")));
    //sep->addStep(new LB_StepCharacteriseCurvature:(*createNewStepInitializeData(NULL)));

    return true;
}

bool LB_PluginManager::loadCanBeAddedFirstStep()
{
    clearCanBeAddedFirstStep();

    CT_StepCanBeAddedFirstSeparator *sep = addNewSeparator(new CT_StepCanBeAddedFirstSeparator());
    //sep->addStep(new NomDeLEtape(*createNewStepInitializeData(NULL)));

    return true;
}

bool LB_PluginManager::loadActions()
{
    clearActions();

    CT_ActionsSeparator *sep = addNewSeparator(new CT_ActionsSeparator(CT_AbstractAction::TYPE_SELECTION));
    //sep->addAction(new NomDeLAction());

    return true;
}

bool LB_PluginManager::loadExporters()
{
    clearExporters();

    CT_StandardExporterSeparator *sep = addNewSeparator(new CT_StandardExporterSeparator("TYPE DE FICHIER"));
    //sep->addExporter(new NomDeLExporter());

    return true;
}

bool LB_PluginManager::loadReaders()
{
    clearReaders();

    CT_StandardReaderSeparator *sep = addNewSeparator(new CT_StandardReaderSeparator("TYPE DE FICHIER"));
    //sep->addReader(new NomDuReader());

    return true;
}

