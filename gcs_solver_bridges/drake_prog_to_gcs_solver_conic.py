import gcs_solver
from gcs_solver import ConicProgram
from pydrake.solvers import ConicStandardForm, MathematicalProgram, ProgramAttribute


def drake_prog_to_gcs_solver_conic(
    prog: MathematicalProgram,
) -> tuple[ConicProgram, ConicStandardForm]:
    drake_conic_form = ConicStandardForm(prog)

    cones = []
    start_end_pair_to_cone_type = {
        start_end_pair: drake_cone_type
        for drake_cone_type, start_end_pair_vect_for_attr in drake_conic_form.attributes_to_start_end_pairs().items()
        for start_end_pair in start_end_pair_vect_for_attr
    }

    last_end = 0
    for (start, end), drake_cone_type in sorted(
        start_end_pair_to_cone_type.items(), key=lambda item: item[0][0]
    ):
        if start != last_end:
            raise ValueError("Cones are not contiguous.")
        cone_dim = end - start
        if drake_cone_type == ProgramAttribute.kLinearEqualityConstraint:
            cones.append(gcs_solver.ZeroCone(cone_dim))
        elif drake_cone_type == ProgramAttribute.kLinearConstraint:
            cones.append(gcs_solver.NonnegativeOrthant(cone_dim))
        elif drake_cone_type == ProgramAttribute.kLorentzConeConstraint:
            cones.append(gcs_solver.LorentzCone(cone_dim))
        elif drake_cone_type == ProgramAttribute.kPositiveSemidefiniteConstraint:
            cones.append(
                gcs_solver.PositiveSemidefiniteCone(
                    cone_dim,
                    gcs_solver.PositiveSemidefiniteConeConstructorType.kDimension,
                )
            )
        else:
            raise ValueError(f"Unsupported cone type: {drake_cone_type}")
        last_end = end
    assert last_end == drake_conic_form.A().shape[0]
    gcs_conic_prog = ConicProgram(
        drake_conic_form.c().toarray().flatten(),
        drake_conic_form.d(),
        drake_conic_form.A().toarray(),
        -drake_conic_form.b().toarray().flatten(),
        cones,
    )
    return gcs_conic_prog, drake_conic_form
