/**
 ******************************************************************************
 *
 * @file       naze32.cpp
 * @author     Tau Labs, http://github.com/TauLabs, Copyright (C) 2013.
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_Naze Afrodevices boards support Plugin
 * @{
 * @brief Plugin to support boards by Afrodevices
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "naze32.h"

/**
 * @brief Naze32::Naze32
 *  This is the Naze32 board definition
 */
Naze32::Naze32(void)
{
    boardType = 0x05;
}

Naze32::~Naze32()
{

}

QString Naze32::shortName()
{
    return QString("naze");
}

QString Naze32::boardDescription()
{
    return QString("naze32 flight control rev. 4 by Afrodevices");
}

//! Return which capabilities this board has
bool Naze32::queryCapabilities(BoardCapabilities capability)
{
    switch(capability) {
    case BOARD_CAPABILITIES_GYROS:
        return true;
    case BOARD_CAPABILITIES_ACCELS:
        return true;
    case BOARD_CAPABILITIES_MAGS:
        return true;
    case BOARD_CAPABILITIES_BAROS:
        return true;
    case BOARD_CAPABILITIES_RADIO:
        return false;
    }
    return false;
}

/**
 * @brief Naze32::getSupportedProtocols
 *  TODO: this is just a stub, we'll need to extend this a lot with multi protocol support
 * @return
 */
QStringList Naze32::getSupportedProtocols()
{

    return QStringList("uavtalk");
}
