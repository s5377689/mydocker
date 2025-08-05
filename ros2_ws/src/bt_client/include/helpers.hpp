#pragma once
#include <QString>

QString readXmlFile(
    const QString & filename,
    const QString & prefix = ""
);

QString buildSingleNodeTree(
    const QString & node_name
);