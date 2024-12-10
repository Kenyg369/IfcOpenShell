﻿/********************************************************************************
*                                                                              *
* This file is part of IfcOpenShell.                                           *
*                                                                              *
* IfcOpenShell is free software: you can redistribute it and/or modify         *
* it under the terms of the Lesser GNU General Public License as published by  *
* the Free Software Foundation, either version 3.0 of the License, or          *
* (at your option) any later version.                                          *
*                                                                              *
* IfcOpenShell is distributed in the hope that it will be useful,              *
* but WITHOUT ANY WARRANTY; without even the implied warranty of               *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 *
* Lesser GNU General Public License for more details.                          *
*                                                                              *
* You should have received a copy of the Lesser GNU General Public License     *
* along with this program. If not, see <http://www.gnu.org/licenses/>.         *
*                                                                              *
********************************************************************************/

#ifndef IFCGEOMMANIFOLDEREPRESENTATION_H
#define IFCGEOMMANIFOLDEREPRESENTATION_H

// ---------below lib is for opencascade( delete to do)-------
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepGProp_Face.hxx>

#include <Poly_Triangulation.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>

#include <TopExp_Explorer.hxx>
#include <BRepTools.hxx>

#include <gp_GTrsf.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
// --------------------------------

#include "../../../ifcgeom/ConversionResult.h"

namespace ifcopenshell {
	namespace geometry {

		using IfcGeom::OpaqueCoordinate;
		using IfcGeom::OpaqueNumber;

		class ManifoldShape : public IfcGeom::ConversionResultShape {
		public:
            ManifoldShape(const TopoDS_Shape& shape)
				: shape_(shape) {}

			const TopoDS_Shape& shape() const { return shape_; }
			operator const TopoDS_Shape& () { return shape_; }

			virtual void Triangulate(ifcopenshell::geometry::Settings settings, const ifcopenshell::geometry::taxonomy::matrix4& place, IfcGeom::Representation::Triangulation* t, int item_id, int surface_style_id) const;
			virtual void Serialize(const ifcopenshell::geometry::taxonomy::matrix4& place, std::string&) const;

			virtual IfcGeom::ConversionResultShape* clone() const {
                return new ManifoldShape(shape_);
			}

			virtual double bounding_box(void*&) const {
				throw std::runtime_error("Not implemented");
			}

			virtual void set_box(void*) {
				throw std::runtime_error("Not implemented");
			}
			
			// ----------1----------
            virtual int surface_genus() const {
                throw std::runtime_error("Not implemented");
            };
            virtual bool is_manifold() const {
                throw std::runtime_error("Not implemented");
            };

			virtual int num_vertices() const {
                throw std::runtime_error("Not implemented");
            };
            virtual int num_edges() const {
                throw std::runtime_error("Not implemented");
            };
            virtual int num_faces() const {
                throw std::runtime_error("Not implemented");
            };

			// @todo this must be something with a virtual dtor so that we can delete it.
            virtual std::pair<OpaqueCoordinate<3>, OpaqueCoordinate<3>> bounding_box() const {
                throw std::runtime_error("Not implemented");
            };
            ;
            //---------------------------


			virtual OpaqueNumber* length();
			virtual OpaqueNumber* area();
			virtual OpaqueNumber* volume();

			virtual OpaqueCoordinate<3> position();
			virtual OpaqueCoordinate<3> axis();
			virtual OpaqueCoordinate<4> plane_equation();

			virtual std::vector<ConversionResultShape*> convex_decomposition();
			virtual ConversionResultShape* halfspaces();
			virtual ConversionResultShape* solid();
			virtual ConversionResultShape* box();

			virtual std::vector<ConversionResultShape*> vertices();
			virtual std::vector<ConversionResultShape*> edges();
			virtual std::vector<ConversionResultShape*> facets();
			//-------------2--------------
            virtual ConversionResultShape* add(ConversionResultShape*) {
                throw std::runtime_error("Not implemented");
            };
            virtual ConversionResultShape* subtract(ConversionResultShape*) {
                throw std::runtime_error("Not implemented");
            };
            virtual ConversionResultShape* intersect(ConversionResultShape*) {
                throw std::runtime_error("Not implemented");
            };

			virtual void map(OpaqueCoordinate<4>& from, OpaqueCoordinate<4>& to) {
                throw std::runtime_error("Not implemented");
            };
            virtual void map(const std::vector<OpaqueCoordinate<4>>& from, const std::vector<OpaqueCoordinate<4>>& to) {
                throw std::runtime_error("Not implemented");
            };
            virtual ConversionResultShape* moved(ifcopenshell::geometry::taxonomy::matrix4::ptr) const {
                throw std::runtime_error("Not implemented");
            };
            //---------------------------

		private:
			TopoDS_Shape shape_;
		};

	}
}

#endif