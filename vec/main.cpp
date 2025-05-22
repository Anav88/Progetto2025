

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
// #include <execution>
#include <iostream>
#include <random>

#include "boids.hpp"

int main() {
  try {
    Par parametres = init_parametres();
    std::vector<Boid> boids(parametres.N);

    sf::RenderWindow window(sf::VideoMode(600, 600), "My window");
    window.setFramerateLimit(60);

    sf::RectangleShape rectangle({400.0f, 400.0f});
    rectangle.setPosition({100.0f, 100.0f});
    rectangle.setOutlineColor(sf::Color::Black);
    rectangle.setOutlineThickness(2.0f);

    add_boid(boids);

    std::vector<sf::ConvexShape> triangles(parametres.N);
    add_triangle(triangles);

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
      }

      window.clear(sf::Color::White);
      window.draw(rectangle);

      evaluate_correction(boids, parametres);
      {
        auto it_b = boids.begin();
        for (auto it = triangles.begin(); it != triangles.end(); ++it, ++it_b) {
          init_tr((*it_b), (*it));
          
          (*it_b).correction();
          (*it_b).limit();
          (*it_b).vel_max();
          
          window.draw(*it);
        }
      }

      window.display();
    }

  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}
