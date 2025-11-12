# main.py

import os
import sys
import glob
import utils
import solver

def main():
    """
    Main function to process all problem files and generate solution files.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # --- Define paths ---
    example_problems_dir = os.path.join(script_dir, "example-problems")
    assignment_problems_dir = os.path.join(script_dir, "problems")
    solutions_dir = os.path.join(script_dir, "solutions")

    os.makedirs(solutions_dir, exist_ok=True)

    # --- Find all problem files in BOTH directories (non-recursive) ---
    problem_files = []

    # Search in example-problems/ 
    example_pattern = os.path.join(example_problems_dir, "problem_*.txt")
    problem_files.extend(glob.glob(example_pattern))

    # Search in problems/ 
    assignment_pattern = os.path.join(assignment_problems_dir, "problem_*.txt")
    problem_files.extend(glob.glob(assignment_pattern))

    if not problem_files:
        print(f"Warning: No problem files found in 'example-problems/' or 'problems/' directories.")
        return

    print(f"Found {len(problem_files)} problem files to process.")

    for problem_file_path in problem_files:
        print(f"Processing: {os.path.basename(problem_file_path)} ...", end=' ')
        
        try:
            problem_data = utils.load_problem_file(problem_file_path)

            if problem_data['type'] == 'CHECK_PLAN':
                solution_text = solver.check_plan(problem_data)
            elif problem_data['type'] == 'FIND_PLAN':
                # TODO: Implement solver.find_plan
                solution_text = solver.find_plan(problem_data)
            else:
                print(f"ERROR: Unknown problem type '{problem_data['type']}'")
                continue

            # Generate solution file name by replacing 'problem_' with 'solution_'
            problem_filename = os.path.basename(problem_file_path)
            solution_filename = problem_filename.replace('problem_', 'solution_', 1)
            solution_file_path = os.path.join(solutions_dir, solution_filename)

            with open(solution_file_path, 'w') as sol_file:
                sol_file.write(solution_text + '\n')

            print(f"Done. Solution saved to {solution_file_path}")

        except Exception as e:
            print(f"ERROR: Failed to process {problem_file_path}. Reason: {e}")

    print("\nProcessing complete.")


if __name__ == "__main__":
    main()
