
def main() -> None:
	N_CONTROL_LAWS:int = 0
	N_K           :int = 0
	IN_FILENAME:str  = "k_gains.txt"
	OUT_FILENAME:str = "main\\k_values.h"

	k_values = list()

	print(f"K values read from: \"{IN_FILENAME}\"")
	with open(IN_FILENAME, "r") as Kval_in:
		lines = Kval_in.readlines()
		values = lines[0].split(" ")
		values = [int(value) for value in values]

		N_CONTROL_LAWS = values[0]
		N_K            = values[1]

		for line in lines[1:]:
			k_values.append([float(value) for value in line.split(" ")])

		print(f"Amount of control laws: {N_CONTROL_LAWS}")
		print(f"Number of K gains:      {N_K}\n")
		for k_val in k_values:
			print(k_val)

	with open(OUT_FILENAME, "+w") as Kval_out:
		Kval_out.write("#pragma once\n\n")
		Kval_out.write(f"const int N_CONTROL_LAWS = {N_CONTROL_LAWS};\n")
		Kval_out.write(f"const int N_K            = {N_K};\n")
		Kval_out.write("\nconst float K_VALUES[N_CONTROL_LAWS][N_K] =\n{\n")
		for i, k_val in enumerate(k_values):
			Kval_out.write("\t{")
			for j, k in enumerate(k_val):
				Kval_out.write(f"{k}{', 'if j<N_K-1 else '}'}")
			Kval_out.write(",\n" if i<N_CONTROL_LAWS-1 else "\n")
		Kval_out.write("};\n")
	print(f"\nK values exported to file: \"{OUT_FILENAME}\"")
	return

if __name__ == "__main__":
	main()
