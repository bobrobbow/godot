/* register_types.cpp */

#include "register_types.h"

#include "core/class_db.h"
#include "road_generator.h"

void register_road_generator_types() {
    ClassDB::register_class<road_generator>();
}

void unregister_road_generator_types() {
   // Nothing to do here in this example.
}
