import random

def generate_funky_maze_with_entrances(filename, width, height):
    # Start with a solid block of walls (1)
    grid = [[1 for _ in range(width)] for _ in range(height)]
    
    # ---------------------------------------------------------
    # 1. GENERATE FUNKY VARIABLE-WIDTH CORRIDORS (Y: 1 to 129)
    # ---------------------------------------------------------
    
    # Core structural nodes to map out the path network
    waypoints = [
        (6, 6),          # Top Left
        (width - 7, 6),  # Top Right
        (width // 2, 40),# Center Upper
        (10, 80),        # Mid Left
        (width - 11, 80),# Mid Right
        (width // 2, 115)# Lower Center Exit
    ]
    
    # Connect waypoints sequentially to guarantee an interconnected path
    for i in range(len(waypoints) - 1):
        x, y = waypoints[i]
        target_x, target_y = waypoints[i+1]
        
        while x != target_x or y != target_y:
            if x < target_x: x += 1
            elif x > target_x: x -= 1
            if y < target_y: y += 1
            elif y > target_y: y -= 1
            
            # Pick a random radius between 2 and 4 (yielding 5 to 9 cells path width)
            radius = random.randint(2, 4)
            
            for cy in range(max(1, y - radius), min(128, y + radius + 1)):
                for cx in range(max(1, x - radius), min(width - 1, x + radius + 1)):
                    grid[cy][cx] = 0

    # Add floating pockets to keep the structure asymmetrical and winding
    for _ in range(12):
        rx = random.randint(10, width - 11)
        ry = random.randint(15, 110)
        radius = random.randint(3, 5)
        for cy in range(max(1, ry - radius), min(128, ry + radius + 1)):
            for cx in range(max(1, rx - radius), min(width - 1, rx + radius + 1)):
                grid[cy][cx] = 0

    # ---------------------------------------------------------
    # 2. CARVE OPEN THE ENTRY SPOTS AT THE TOP (Y: 1 to 4)
    # ---------------------------------------------------------
    # We blow wide open slots at the top wall layer so falling particles
    # can funnel straight into the maze network seamlessly.
    entrance_width = 12
    
    # Left Entrance Funnel
    for y in range(1, 5):
        for x in range(6, 6 + entrance_width):
            grid[y][x] = 0
            
    # Right Entrance Funnel
    for y in range(1, 5):
        for x in range(width - 6 - entrance_width, width - 6):
            grid[y][x] = 0

    # ---------------------------------------------------------
    # 3. DESIGN THE PARTICLE RESERVOIR BASIN (Y: 130 to 204)
    # ---------------------------------------------------------
    basin_top = 130
    basin_bottom = height - 4 
    
    for y in range(basin_top, basin_bottom + 1):
        for x in range(1, width - 1):
            grid[y][x] = 0

    # Inject solid floor foundation at the very base
    for y in range(basin_bottom + 1, height - 1):
        for x in range(1, width - 1):
            grid[y][x] = 1

    # ---------------------------------------------------------
    # 4. FILL THE RESERVOIR WITH PARTICLES (2)
    # ---------------------------------------------------------
    particle_fill_top = 165
    for y in range(particle_fill_top, basin_bottom + 1):
        for x in range(4, width - 4):
            grid[y][x] = 2

    # ---------------------------------------------------------
    # 5. EXPORT STRUCT TABLE
    # ---------------------------------------------------------
    with open(filename, 'w') as f:
        f.write("FLOWBOX_GRID\n")
        for y in range(height):
            f.write(" ".join(str(grid[y][x]) for x in range(width)) + "\n")
            
    print(f"Generated funky {filename} successfully with drop entry gates.")

if __name__ == "__main__":
    generate_funky_maze_with_entrances("grid_2.txt", 80, 205)
