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

#include "RPIMoCap/Server/camerasettingswidget.h"
#include "ui_camerasettingswidget.h"

#include <QMetaProperty>

namespace RPIMoCap {

CameraSettingsWidget::CameraSettingsWidget(QUuid id, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::CameraSettingsWidget)
{
    ui->setupUi(this);
    ui->cameraid->setText(id.toString(QUuid::StringFormat::WithoutBraces));
}

CameraSettingsWidget::~CameraSettingsWidget()
{
    delete ui;
}

Visualization::ExtrinsicWidget *CameraSettingsWidget::extrinsic()
{
    return ui->extrinsic;
}


}
