/****************************************************************************
** Meta object code from reading C++ file 'QtGnuplotApplication.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "QtGnuplotApplication.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtGnuplotApplication.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_QtGnuplotApplication_t {
    QByteArrayData data[6];
    char stringdata0[78];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QtGnuplotApplication_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QtGnuplotApplication_t qt_meta_stringdata_QtGnuplotApplication = {
    {
QT_MOC_LITERAL(0, 0, 20), // "QtGnuplotApplication"
QT_MOC_LITERAL(1, 21, 15), // "windowDestroyed"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 6), // "object"
QT_MOC_LITERAL(4, 45, 16), // "enterPersistMode"
QT_MOC_LITERAL(5, 62, 15) // "exitPersistMode"

    },
    "QtGnuplotApplication\0windowDestroyed\0"
    "\0object\0enterPersistMode\0exitPersistMode"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QtGnuplotApplication[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x0a /* Public */,
       1,    0,   37,    2, 0x2a /* Public | MethodCloned */,
       4,    0,   38,    2, 0x0a /* Public */,
       5,    0,   39,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QObjectStar,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QtGnuplotApplication::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QtGnuplotApplication *_t = static_cast<QtGnuplotApplication *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->windowDestroyed((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        case 1: _t->windowDestroyed(); break;
        case 2: _t->enterPersistMode(); break;
        case 3: _t->exitPersistMode(); break;
        default: ;
        }
    }
}

const QMetaObject QtGnuplotApplication::staticMetaObject = {
    { &QApplication::staticMetaObject, qt_meta_stringdata_QtGnuplotApplication.data,
      qt_meta_data_QtGnuplotApplication,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *QtGnuplotApplication::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QtGnuplotApplication::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_QtGnuplotApplication.stringdata0))
        return static_cast<void*>(const_cast< QtGnuplotApplication*>(this));
    if (!strcmp(_clname, "QtGnuplotEventReceiver"))
        return static_cast< QtGnuplotEventReceiver*>(const_cast< QtGnuplotApplication*>(this));
    return QApplication::qt_metacast(_clname);
}

int QtGnuplotApplication::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QApplication::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
