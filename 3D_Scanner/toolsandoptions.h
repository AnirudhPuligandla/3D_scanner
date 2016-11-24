#ifndef TOOLSANDOPTIONS_H
#define TOOLSANDOPTIONS_H

#include <QPushButton>
#include <QObject>
#include <QWidget>

class ToolsAndOptions : public QWidget
{
    Q_OBJECT

private:
    //start button
    QPushButton *startButton_;

public:
    explicit ToolsAndOptions(QWidget *parent = 0);
    //getStartButton Function
    QPushButton* getStartButton();

public slots:

    //start button pressed() handler
    void on_startButton_pressed();
};

#endif // TOOLSANDOPTIONS_H
