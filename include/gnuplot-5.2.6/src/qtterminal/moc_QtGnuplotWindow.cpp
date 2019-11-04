/****************************************************************************
** Meta object code from reading C++ file 'QtGnuplotWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "QtGnuplotWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtGnuplotWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_QtGnuplotWindow_t {
    QByteArrayData data[11];
    char stringdata0[147];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QtGnuplotWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QtGnuplotWindow_t qt_meta_stringdata_QtGnuplotWindow = {
    {
QT_MOC_LITERAL(0, 0, 15), // "QtGnuplotWindow"
QT_MOC_LITERAL(1, 16, 16), // "on_setStatusText"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 6), // "status"
QT_MOC_LITERAL(4, 41, 12), // "on_keyAction"
QT_MOC_LITERAL(5, 54, 5), // "print"
QT_MOC_LITERAL(6, 60, 11), // "exportToPdf"
QT_MOC_LITERAL(7, 72, 13), // "exportToImage"
QT_MOC_LITERAL(8, 86, 11), // "exportToSvg"
QT_MOC_LITERAL(9, 98, 18), // "showSettingsDialog"
QT_MOC_LITERAL(10, 117, 29) // "settingsSelectBackgroundColor"

    },
    "QtGnuplotWindow\0on_setStatusText\0\0"
    "status\0on_keyAction\0print\0exportToPdf\0"
    "exportToImage\0exportToSvg\0showSettingsDialog\0"
    "settingsSelectBackgroundColor"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QtGnuplotWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x0a /* Public */,
       4,    0,   57,    2, 0x0a /* Public */,
       5,    0,   58,    2, 0x08 /* Private */,
       6,    0,   59,    2, 0x08 /* Private */,
       7,    0,   60,    2, 0x08 /* Private */,
       8,    0,   61,    2, 0x08 /* Private */,
       9,    0,   62,    2, 0x08 /* Private */,
      10,    0,   63,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QtGnuplotWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QtGnuplotWindow *_t = static_cast<QtGnuplotWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_setStatusText((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->on_keyAction(); break;
        case 2: _t->print(); break;
        case 3: _t->exportToPdf(); break;
        case 4: _t->exportToImage(); break;
        case 5: _t->exportToSvg(); break;
        case 6: _t->showSettingsDialog(); break;
        case 7: _t->settingsSelectBackgroundColor(); break;
        default: ;
        }
    }
}

const QMetaObject QtGnuplotWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_QtGnuplotWindow.data,
      qt_meta_data_QtGnuplotWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *QtGnuplotWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QtGnuplotWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_QtGnuplotWindow.stringdata0))
        return static_cast<void*>(const_cast< QtGnuplotWindow*>(this));
    if (!strcmp(_clname, "QtGnuplotEventReceiver"))
        return static_cast< QtGnuplotEventReceiver*>(const_cast< QtGnuplotWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int QtGnuplotWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
