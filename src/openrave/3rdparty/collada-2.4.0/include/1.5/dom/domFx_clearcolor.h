#ifndef __dom150Fx_clearcolor_h__
#define __dom150Fx_clearcolor_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>


class DAE;
namespace ColladaDOM150 {

class domFx_clearcolor : public daeElement
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_CLEARCOLOR; }
	static daeInt ID() { return 134; }
	virtual daeInt typeID() const { return ID(); }
protected:  // Attribute
	xsNonNegativeInteger attrIndex;


public:	//Accessors and Mutators
	/**
	 * Gets the index attribute.
	 * @return Returns a xsNonNegativeInteger of the index attribute.
	 */
	xsNonNegativeInteger getIndex() const { return attrIndex; }
	/**
	 * Sets the index attribute.
	 * @param atIndex The new value for the index attribute.
	 */
	void setIndex( xsNonNegativeInteger atIndex ) { attrIndex = atIndex; }

	/**
	 * Gets the value of this element.
	 * @return a domFx_color of the value.
	 */
	domFx_color& getValue() { return _value; }
	/**
	 * Sets the _value of this element.
	 * @param val The new value for this element.
	 */
	void setValue( const domFx_color& val ) { _value = val; }

protected:  // Value
	/**
	 * The domFx_color value of the text data of this element. 
	 */
	domFx_color _value;
protected:
	/**
	 * Constructor
	 */
	domFx_clearcolor(DAE& dae) : daeElement(dae), attrIndex(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_clearcolor() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_clearcolor &operator=( const domFx_clearcolor &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static DLLSPEC daeElementRef create(DAE& dae);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static DLLSPEC daeMetaElement* registerElement(DAE& dae);
};


} // ColladaDOM150
#endif
