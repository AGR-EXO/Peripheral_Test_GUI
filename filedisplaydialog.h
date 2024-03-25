#ifndef FILEDISPLAYDIALOG_H
#define FILEDISPLAYDIALOG_H


//#include <QtWidgets>
#include <QDialog>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QDebug>

class FileDisplayDialog: public QDialog
{
    Q_OBJECT

    public:
    FileDisplayDialog(QWidget *parent = nullptr);
    ~FileDisplayDialog();

public slots:
    void displayFileContents(const QString &fileName);

private:
    QTextEdit *textEdit;
};

#endif // FILEDISPLAYDIALOG_H

