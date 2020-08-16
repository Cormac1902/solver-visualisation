//
// Created by cormac on 16/08/2020.
//

#ifndef INC_3DVIS_EDGECOLOUR_H
#define INC_3DVIS_EDGECOLOUR_H


#include <vtkType.h>
#include <vtkColor.h>

class EdgeColour {
    vtkIdType id;
    unsigned duplication;
    vtkColor4ub colour;
public:
    explicit EdgeColour(vtkColor4ub color4Ub = *new vtkColor4ub) : id(0), duplication(0), colour(color4Ub) {};

    [[nodiscard]] vtkIdType getId() const {
        return id;
    }

    void setId(vtkIdType vtkId) {
        id = vtkId;
    }

    [[nodiscard]] unsigned int getDuplication() const {
        return duplication;
    }

    void setDuplication(unsigned int duplicated) {
        duplication = duplicated;
    }

    [[nodiscard]] vtkColor4ub &getColour() {
        return colour;
    }

    [[nodiscard]] const unsigned char* getColourData() {
        return colour.GetData();
    }

    void setColour(const vtkColor4ub vtkColour) {
        colour = vtkColour;
    }

    void setColour(const float highestEdgeDuplication) {
        colour[3] = 255 * ((float) duplication / highestEdgeDuplication);
    }
};


#endif //INC_3DVIS_EDGECOLOUR_H
