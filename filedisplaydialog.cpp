#include "filedisplaydialog.h"

FileDisplayDialog::FileDisplayDialog(QWidget *parent)
    :QDialog(parent)
{
    // Set up the layout
    QVBoxLayout *layout = new QVBoxLayout(this);
    textEdit = new QTextEdit(this);
    layout->addWidget(textEdit);

    QPushButton *closeButton = new QPushButton("Close", this);
    connect(closeButton, &QPushButton::clicked, this, &FileDisplayDialog::accept);
    layout->addWidget(closeButton);

    setLayout(layout);
}

FileDisplayDialog::~FileDisplayDialog()
{
   // delete ui;
}

void FileDisplayDialog::displayFileContents(const QString &fileName)
{
    // Open and read the contents of the file
    QFile file(fileName);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream stream(&file);
        QString content = stream.readAll();
        textEdit->setPlainText(content);
        file.close();
        show();  // Show the dialog
    }
    else
    {
        qDebug() << "Error opening file:" << file.errorString();
    }
}
