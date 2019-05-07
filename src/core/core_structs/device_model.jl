abstract type AbstractDeviceFormulation end

function _validate_device_formulation(device_model::Type{D}) where {D <: Union{AbstractDeviceFormulation, PSY.Device}}
    
    if !isconcretetype(device_model)
        throw(ArgumentError( "the device model must containt only concrete types, $(device_model) is an Abstract Type"))
    end

end

mutable struct DeviceModel{D <: PSY.Device,
                           B <: PSI.AbstractDeviceFormulation}
    device::Type{D}
    formulation::Type{B}

    function DeviceModel(device::Type{D},
                         formulation::Type{B}) where {D <: PSY.Device,
                                                      B <: PSI.AbstractDeviceFormulation}
                        
                        _validate_device_formulation(D)
                        _validate_device_formulation(B)
                        new{D,B}(D,B)

    end

end

mutable struct InitialCondition{T <: Union{PJ.ParameterRef, Float64}}
    device::PSY.Device
    value::T
end
