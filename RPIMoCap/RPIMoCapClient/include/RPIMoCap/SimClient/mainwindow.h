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
#include "RPIMoCap/SimClient/simcamera.h"

#include <QMainWindow>
#include <QThread>

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
    void onRotationChanged(QUuid clientId, Eigen::Vector3d rVec);
    void onTranslationChanged(QUuid clientId, Eigen::Vector3d tVec);
    void onfpsChanged(QUuid clientId, int64_t fps);
    void openProject();
    void saveProject();
    void createClient();
    void removeClient();

private:
    void closeEvent(QCloseEvent *event) override;

    void addClient(const QUuid &id, const RPIMoCap::Camera::Intrinsics &params,
                   const Eigen::Vector3d &rVec, const Eigen::Vector3d& tVec);
    void clearClients();

    Ui::MainWindow *m_ui = nullptr;

    SimScene &m_scene;

    struct ClientData {
        QUuid id;
        std::shared_ptr<SimCamera> camera;
        QSharedPointer<Client> client;
        QWidget* widget = nullptr;
        QThread* thread = nullptr;

        void clear()
        {
            camera.reset();
            client.reset();
            thread->quit();
            thread->wait();
            widget->deleteLater();
            thread->deleteLater();
        }
    };

    QVector<ClientData> m_clients;

    void drawFrame();
};

}
