#ifndef LB_PLUGINMANAGER_H
#define LB_PLUGINMANAGER_H

#include "ct_abstractstepplugin.h"

class LB_PluginManager : public CT_AbstractStepPlugin
{
public:
    LB_PluginManager();
    ~LB_PluginManager();

    QString getPluginURL() const {return QString("http://rdinnovation.onf.fr:8080/projects/PLUGINS-PROJECT-NAME-HERE");}

protected:

    bool loadGenericsStep();
    bool loadOpenFileStep();
    bool loadCanBeAddedFirstStep();
    bool loadActions();
    bool loadExporters();
    bool loadReaders();

};

#endif // LB_PLUGINMANAGER_H
