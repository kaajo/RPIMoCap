/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2019 Miroslav Krajicek.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "simscene.h"

#include <RPIMoCap/ClientLib/client.h>

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

namespace RPIMoCap::SimClient {

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(SimScene &scene, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateValue();

    void on_addClientButton_clicked();

    void on_removeClientButton_clicked();

private:
    void closeEvent(QCloseEvent *event) override;

    Ui::MainWindow *ui;

    SimScene &m_scene;

    QVector<QSharedPointer<Client>> m_clients;
    QVector<QWidget*> m_clientWidgets;
    QVector<QThread*> m_clientThreads;
};

}
