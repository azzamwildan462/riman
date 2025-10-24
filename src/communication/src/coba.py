import requests
import keyboard
import json

def get_request(url):
    """Melakukan GET request"""
    try:
        response = requests.get(url)
        print(f"\n[GET] Status: {response.status_code}")
        print(f"Response: {response.text[:200]}")  # Tampilkan 200 karakter pertama
        return response
    except Exception as e:
        print(f"Error GET: {e}")

def post_request(url, data):
    """Melakukan POST request"""
    try:
        response = requests.post(url, json=data)
        print(f"\n[POST] Status: {response.status_code}")
        print(f"Response: {response.text[:200]}")
        return response
    except Exception as e:
        print(f"Error POST: {e}")

def main():
    print("=== Program GET/POST dengan Kontrol Keyboard ===")
    print("Tekan 'g' untuk GET request")
    print("Tekan 'p' untuk POST request")
    print("Tekan 'q' untuk keluar")
    print("=" * 50)
    
    default_url_1 = "http://10.7.101.213/cmd/nav"
    default_url_2 = "http://10.7.101.213/cmd/nav"
    
    while True:
        if keyboard.is_pressed('g'):
            print("\n[Trigger: GET Request]")
            get_request(default_url_1)
            keyboard.wait('g', suppress=True)  # Tunggu tombol dilepas
            
        elif keyboard.is_pressed('1'):
            print("\n[Trigger: POST Request]")
            data = {
                "x": 0, "y": 0, "theta": 0
            }
            post_request(default_url_2, data)
            keyboard.wait('p', suppress=True)

        elif keyboard.is_pressed('2'):
            print("\n[Trigger: POST Request]")
            data = {
                "x": 1, "y": 0, "theta": 0
            }
            post_request(default_url_2, data)
            keyboard.wait('p', suppress=True)

        elif keyboard.is_pressed('3'):
            print("\n[Trigger: POST Request]")
            data = {
                "x": 0, "y": 1, "theta": 0
            }
            post_request(default_url_2, data)
            keyboard.wait('p', suppress=True)
            
        elif keyboard.is_pressed('q'):
            print("\nKeluar dari program...")
            break

if __name__ == "__main__":
    main()