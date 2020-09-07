//
// Created by cormac on 16/08/2020.
//

#ifndef INC_3DVIS_EDGECOLOUR_H
#define INC_3DVIS_EDGECOLOUR_H

#include <vtkType.h>
#include <vtkSmartPointer.h>
#include <vtkColor.h>
#include <vtkNamedColors.h>

class EdgeColour {
    vtkIdType id;
    unsigned duplication;
    vtkColor4ub colour;
public:
    explicit EdgeColour() : id(0), duplication(0), colour() {};

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

    [[nodiscard]] vtkColor4ub getColour() {
        return colour;
    }

    [[nodiscard]] unsigned char* getColourData() {
        return colour.GetData();
    }

    void setColour(vtkColor4ub vtkColour) {
        colour = vtkColour;
    }

    void setColour(const float highestEdgeDuplication) {
        colour.SetAlpha(230 * ((float) duplication / highestEdgeDuplication));
    }

    [[nodiscard]] bool new_edge() const {
        return id == 0;
    }
};


#endif //INC_3DVIS_EDGECOLOUR_H
