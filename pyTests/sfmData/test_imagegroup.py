"""
Collection of unit tests for the ImageGroup, ImageSet, and ImageSequence classes.
"""

from pyalicevision import sfmData as av


def test_imageset_type_conversion_roundtrip():
    """ Test converting string to ImageGroup Type enum, and ImageGroup Type enum to string. """
    # Get Type enum value using stringToType first
    type_value = av.ImageGroup.stringToType("ImageSet")

    # Verify it returns a valid type object
    assert type_value is not None, "stringToType should return a valid type"

    # Verify round-trip conversion
    type_str = av.ImageGroup.typeToString(type_value)
    assert isinstance(type_str, str), "typeToString should return a string"
    assert type_str == "ImageSet", "Round-trip conversion should preserve type"


def test_imagegroup_create_factory():
    """ Test creating ImageGroup using the factory method. """
    type_value = av.ImageGroup.stringToType("ImageSet")
    image_group = av.ImageGroup.create(type_value)
    
    assert image_group is not None, "Factory should create a valid ImageGroup object"
    # Verify the type by converting back to string
    actual_type_str = av.ImageGroup.typeToString(image_group.getType())
    assert actual_type_str == "ImageSet", "Created ImageGroup should have Type ImageSet"


def test_imageset_default_constructor():
    """ Test creating an ImageSet object with default parameters. """
    image_set = av.ImageSet()
    
    assert image_set is not None, "ImageSet default constructor should succeed"
    # Verify the type by converting to string
    type_str = av.ImageGroup.typeToString(image_set.getType())
    assert type_str == "ImageSet", "ImageSet should have Type ImageSet"


def test_imageset_get_type():
    """ Test getting the type of an ImageSet object. """
    image_set = av.ImageSet()
    type_value = image_set.getType()
    
    # Verify by converting to string
    type_str = av.ImageGroup.typeToString(type_value)
    assert type_str == "ImageSet", "ImageSet.getType() should return ImageSet type"


def test_imageset_clone():
    """ Test cloning an ImageSet object. """
    image_set1 = av.ImageSet()
    image_set2 = image_set1.clone()
    
    assert image_set2 is not None, "Clone should return a valid object"
    assert image_set1 == image_set2, "Cloned ImageSet should be equal to the original"
    assert image_set1.getType() == image_set2.getType(), \
        "Cloned ImageSet should have the same type"


def test_imageset_equality_operators():
    """ Test comparing two ImageSet objects using the '==' and '!=' operators. """
    image_set1 = av.ImageSet()
    image_set2 = av.ImageSet()

    # Since ImageSet currently has no distinguishing properties,
    # two default instances are equal
    assert image_set1 == image_set2, "Two default ImageSet objects should be equal"
    assert not (image_set1 != image_set2), \
        "Two default ImageSet objects should not be unequal"


def test_imagegroup_polymorphism():
    """ Test using ImageSet through ImageGroup interface (polymorphism). """
    # Create via factory
    type_value = av.ImageGroup.stringToType("ImageSet")
    image_group = av.ImageGroup.create(type_value)
    
    # Should be able to call ImageGroup methods
    type_str = av.ImageGroup.typeToString(image_group.getType())
    assert type_str == "ImageSet", "Polymorphic getType() should work correctly"
    
    # Clone should work through base class interface
    cloned = image_group.clone()
    assert cloned is not None, "Polymorphic clone should work"
    cloned_type_str = av.ImageGroup.typeToString(cloned.getType())
    assert cloned_type_str == "ImageSet", "Cloned object should have correct type"


def test_imageset_from_factory_equality():
    """ Test that ImageSets created different ways are equal. """
    image_set_direct = av.ImageSet()
    type_value = av.ImageGroup.stringToType("ImageSet")
    image_set_factory = av.ImageGroup.create(type_value)
    
    # Both should be equal since they have the same type
    assert image_set_direct == image_set_factory, \
        "ImageSet created directly should equal one created via factory"


# ImageSequence Tests

def test_imagesequence_string_to_type_round_trip():
    """ Test converting string to ImageSequence Type enum and ImageSequence Type enum to string. """
    type_value = av.ImageGroup.stringToType("ImageSequence")

    # Verify it returns a valid type object
    assert type_value is not None, "stringToType should return a valid type"

    # Verify round-trip conversion
    type_str = av.ImageGroup.typeToString(type_value)
    assert isinstance(type_str, str), "typeToString should return a string"
    assert type_str == "ImageSequence", "Round-trip conversion should preserve type"


def test_imagesequence_default_constructor():
    """ Test creating an ImageSequence object with default parameters. """
    image_seq = av.ImageSequence()
    
    assert image_seq is not None, "ImageSequence default constructor should succeed"
    # Verify the type by converting to string
    type_str = av.ImageGroup.typeToString(image_seq.getType())
    assert type_str == "ImageSequence", "ImageSequence should have Type ImageSequence"


def test_imagesequence_get_type():
    """ Test getting the type of an ImageSequence object. """
    image_seq = av.ImageSequence()
    type_value = image_seq.getType()
    
    # Verify by converting to string
    type_str = av.ImageGroup.typeToString(type_value)
    assert type_str == "ImageSequence", "ImageSequence.getType() should return ImageSequence type"


def test_imagesequence_clone():
    """ Test cloning an ImageSequence object. """
    image_seq1 = av.ImageSequence()
    image_seq2 = image_seq1.clone()
    
    assert image_seq2 is not None, "Clone should return a valid object"
    assert image_seq1 == image_seq2, "Cloned ImageSequence should be equal to the original"
    assert image_seq1.getType() == image_seq2.getType(), \
        "Cloned ImageSequence should have the same type"


def test_imagesequence_equality_operators():
    """ Test comparing two ImageSequence objects using the '==' and  '!=' operators. """
    image_seq1 = av.ImageSequence()
    image_seq2 = av.ImageSequence()

    # Since ImageSequence currently has no distinguishing properties,
    # two default instances are equal
    assert image_seq1 == image_seq2, "Two default ImageSequence objects should be equal"
    assert not (image_seq1 != image_seq2), \
        "Two default ImageSequence objects should not be unequal"


def test_imagesequence_from_factory():
    """ Test creating ImageSequence using the factory method. """
    type_value = av.ImageGroup.stringToType("ImageSequence")
    image_seq_factory = av.ImageGroup.create(type_value)

    assert image_seq_factory is not None, "Factory should create a valid ImageSequence object"
    # Verify the type by converting back to string
    actual_type_str = av.ImageGroup.typeToString(image_seq_factory.getType())
    assert actual_type_str == "ImageSequence", "Created ImageGroup should have Type ImageSequence"

    image_seq_direct = av.ImageSequence()
    # Both should be equal since they have the same type
    assert image_seq_direct == image_seq_factory, \
        "ImageSequence created directly should equal one created via factory"


# Cross-type Tests

def test_cross_type_comparison():
    """ Test comparing different ImageGroup types. """
    image_set = av.ImageSet()
    image_seq = av.ImageSequence()

    assert image_set != image_seq, "ImageSet and ImageSequence should not be equal"
    assert not (image_set == image_seq), "ImageSet should not equal ImageSequence"

    # Verify they have different types
    set_type_str = av.ImageGroup.typeToString(image_set.getType())
    seq_type_str = av.ImageGroup.typeToString(image_seq.getType())

    assert set_type_str == "ImageSet", "ImageSet should have ImageSet type"
    assert seq_type_str == "ImageSequence", "ImageSequence should have ImageSequence type"


def test_imagegroup_factory_creates_correct_types():
    """ Test that factory creates different concrete types correctly. """
    # Create ImageSet via factory
    set_type = av.ImageGroup.stringToType("ImageSet")
    image_set = av.ImageGroup.create(set_type)
    
    # Create ImageSequence via factory
    seq_type = av.ImageGroup.stringToType("ImageSequence")
    image_seq = av.ImageGroup.create(seq_type)
    
    # Verify types
    assert av.ImageGroup.typeToString(image_set.getType()) == "ImageSet"
    assert av.ImageGroup.typeToString(image_seq.getType()) == "ImageSequence"
    
    # Verify they're not equal
    assert image_set != image_seq, "Different types should not be equal"
