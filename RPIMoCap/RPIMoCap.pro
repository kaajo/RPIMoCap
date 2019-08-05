TEMPLATE = subdirs

SUBDIRS += \
    RPIMoCapBase \
    RPIMoCapClient \
    RPIMoCapServer

RPIMoCapClient.depends = RPIMoCapBase
RPIMoCapServer.depends = RPIMoCapBase
