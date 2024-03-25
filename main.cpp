#include "mainwindow.h"

#include <QApplication>
#include <QDateTime>
#include <QFile>
//#include <QTextStream>


QtMessageHandler originalHandler = nullptr;

void logToFile(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    // Get the current date and time
    QDateTime currentDateTime = QDateTime::currentDateTime();
    // Format the date and time to include in the file name
    QString dateTimeString = currentDateTime.toString("yyyyMMdd_hhmmss");

    QString message = qFormatLogMessage(type, context, msg);
    QString filename="log_"+dateTimeString+".txt";
    const char *filenameChar = filename.toLocal8Bit().data();
    static FILE *f = fopen(filenameChar, "a");
    fprintf(f, "%s\n", qPrintable(message));
    fflush(f);

    if (originalHandler)
        originalHandler(type, context, msg);
}

int main(int argc, char *argv[])
{
    originalHandler = qInstallMessageHandler(logToFile);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
