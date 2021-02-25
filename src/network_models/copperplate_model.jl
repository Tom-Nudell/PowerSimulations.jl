function copper_plate(
    optimization_container::OptimizationContainer,
    expression::Symbol,
    bus_count::Int,
)
    time_steps = model_time_steps(optimization_container)
    remove_undef!(optimization_container.expressions[expression])

    constraint_val = JuMPConstraintArray(undef, time_steps)
    assign_constraint!(optimization_container, "CopperPlateBalance", constraint_val)

    @info "Total operation count before $(optimization_container.JuMPmodel.operator_counter)"
    for t in time_steps
        @info "Total operation count $(optimization_container.JuMPmodel.operator_counter)"
        constraint_val[t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            sum(optimization_container.expressions[expression].data[i, t] for i in 1:bus_count) == 0
        )
    end

    return
end
